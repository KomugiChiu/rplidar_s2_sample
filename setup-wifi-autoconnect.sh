#!/usr/bin/env bash
#
# setup-wifi-autoconnect.sh — provision unattended WiFi with a static address
# on a clean Ubuntu host.
#
# Connects one interface to one SSID at boot. Interfaces are tried in the
# order given by --ifaces and only the first one found is ever brought up;
# the others are forced down. That guarantees the static address is never
# held by two interfaces at once (which would cause ARP conflicts).
#
# Usage:
#   sudo ./setup-wifi-autoconnect.sh [options]
#
#   --ssid and --psk may be omitted; the script prompts for them instead.
#   The passphrase is read without echo. Pass them as flags when running
#   unattended (CI, ansible, first-boot scripts).
#
# Options:
#   --ssid SSID         WiFi SSID                     (prompted for if omitted)
#   --psk PSK           WPA passphrase, 8-63 chars    (prompted for if omitted)
#   --ip ADDR/PREFIX    static address         (default 192.168.50.200/24)
#   --gateway GW        default gateway        (default 192.168.50.1)
#   --ifaces "A B C"    interface priority list, first match wins
#                       (default "wlp1s0 wlxe8de270d57d1")
#   --nm MODE           networkmanager handling: unmanage | keep | stop
#                                                       (default unmanage)
#   --dhcpcd MODE       dhcpcd handling: deny | keep    (default deny)
#                       "deny" adds the target interfaces to denyinterfaces
#                       in /etc/dhcpcd.conf so dhcpcd does not put a lease on
#                       an interface this script addresses itself
#   --dhcp              use DHCP instead of a static address
#   --dry-run           print what would change, touch nothing
#   --uninstall         remove the service, runtime script and config
#   -h, --help          show this help
#
# Address mode: --dhcp selects DHCP; --ip selects static. With neither and a
# terminal attached, the script asks. With neither and no terminal it stops
# and asks for one of the two, so unattended runs are never ambiguous.
#
# Installs on the target host:
#   /usr/local/bin/wifi-autoconnect.sh        runtime helper (supervised)
#   /etc/systemd/system/wifi-autoconnect.service
#   /etc/wpa_supplicant/wpa_supplicant.conf
#
# wpa_supplicant is looked up at /usr/bin/wpa_supplicant first, then at
# /usr/sbin/wpa_supplicant, which is where the distro "wpasupplicant" package
# installs it. A candidate only counts if it actually runs. If neither is
# usable the package is installed (or reinstalled); /usr/local/bin is the
# last resort, for a build the operator maintains by hand.
#
set -euo pipefail

SSID=""
PSK=""
MODE=""                       # "" (unset) | static | dhcp
IP_SET=0                      # was --ip given explicitly?
STATIC_IP="192.168.50.200/24"
GATEWAY="192.168.50.1"
IFACES="wlp1s0 wlxe8de270d57d1"
NM_MODE="unmanage"
DHCPCD_MODE="deny"
DRYRUN=0
UNINSTALL=0

RUNTIME=/usr/local/bin/wifi-autoconnect.sh
UNIT=/etc/systemd/system/wifi-autoconnect.service
WPA_CONF=/etc/wpa_supplicant/wpa_supplicant.conf
SERVICE_NAME=wifi-autoconnect.service

# dhcpcd configuration and the markers used to manage our line in it.
DHCP_CONF=/etc/dhcpcd.conf
DHCPCD_MARK="# setup-wifi-autoconnect: "
DHCPCD_BEGIN="# BEGIN setup-wifi-autoconnect"
DHCPCD_END="# END setup-wifi-autoconnect"

say()  { printf '\033[1;34m::\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31mxx\033[0m %s\n' "$*" >&2; exit 1; }

run() {
  if [ "$DRYRUN" -eq 1 ]; then
    printf '\033[0;90m  would run: %s\033[0m\n' "$*"
  else
    "$@"
  fi
}

usage() { sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; $d'; }
# --------------------------------------------------------------------------
# dhcpcd coexistence
#
# dhcpcd would put a DHCP lease on the same interface this script is about to
# address, leaving the interface with two addresses and the host with two
# default routes. "denyinterfaces" in /etc/dhcpcd.conf is what prevents that.
# dhcpcd 10.x has no "include" directive — its parser rejects it outright —
# so the configuration file itself has to be edited.
#
# A denyinterfaces line that was already there is preserved: its interfaces
# are merged into ours, and the original line is commented out with a marker
# so that --uninstall can restore the file exactly.
# --------------------------------------------------------------------------
dhcpcd_present() {
  command -v dhcpcd >/dev/null 2>&1 \
    || systemctl list-unit-files 2>/dev/null | grep -q '^dhcpcd\.service'
}

# Everything that should end up denied: what the file already lists, both
# active lines and ones a previous run commented out, plus our interfaces.
dhcpcd_denied_list() {
  {
    sed -n 's/^[[:space:]]*denyinterfaces[[:space:]]\+//p' "$DHCP_CONF" 2>/dev/null
    sed -n "s/^${DHCPCD_MARK}[[:space:]]*denyinterfaces[[:space:]]\+//p" \
      "$DHCP_CONF" 2>/dev/null
    for i in $IFACES; do echo "$i"; done
  } | tr -s '[:space:]' '\n' | grep -v '^$' | sort -u | tr '\n' ' '
}

# Rewrite the file with exactly one active denyinterfaces line. The original
# active line becomes a marked comment; the marked comment from a previous
# run is carried over untouched.
dhcpcd_write_conf() {
  local list="$1" tmp
  tmp=$(mktemp) || return 1
  awk -v mark="$DHCPCD_MARK" -v begin="$DHCPCD_BEGIN" -v end="$DHCPCD_END" \
      -v list="$list" '
    $0 == begin { inblk = 1; next }
    $0 == end   { inblk = 0; next }
    inblk { next }
    index($0, mark) == 1 { print; next }
    $1 == "denyinterfaces" { print mark $0; next }
    { print }
    END {
      gsub(/[ \t]+$/, "", list)
      print begin
      print "denyinterfaces " list
      print end
    }
  ' "$DHCP_CONF" >"$tmp" 2>/dev/null \
    && cat "$tmp" >"$DHCP_CONF" && rm -f "$tmp"
}

# Ask dhcpcd to re-read its configuration by restarting it. A running
# instance keeps the interfaces it already manages: SIGHUP only makes it
# rebind them, it does not re-evaluate denyinterfaces, so the rule would
# otherwise not take effect until the next boot.
#
# dhcpcd.conf usually carries "persistent", which makes the daemon leave its
# addresses in place when it exits, so the wired link survives the restart.
# Without that option the link does blip, so warn before doing it.
dhcpcd_restart() {
  if ! grep -qE '^[[:space:]]*persistent' "$DHCP_CONF" 2>/dev/null; then
    warn "dhcpcd.conf has no 'persistent' option; restarting dhcpcd will drop"
    warn "  the addresses it holds for a moment, which may break this session"
  fi
  systemctl restart dhcpcd >/dev/null 2>&1
}

# Undo dhcpcd_write_conf: drop the managed block and un-comment what it
# commented out, leaving the file as it was.
dhcpcd_restore_conf() {
  local tmp
  [ -f "$DHCP_CONF" ] || return 0
  tmp=$(mktemp) || return 1
  awk -v mark="$DHCPCD_MARK" -v begin="$DHCPCD_BEGIN" -v end="$DHCPCD_END" '
    $0 == begin { inblk = 1; next }
    $0 == end   { inblk = 0; next }
    inblk { next }
    index($0, mark) == 1 { sub("^" mark, "", $0); print; next }
    { print }
  ' "$DHCP_CONF" >"$tmp" 2>/dev/null \
    && cat "$tmp" >"$DHCP_CONF" && rm -f "$tmp"
}


# --------------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --ssid)     SSID=${2:?--ssid needs a value}; shift 2 ;;
    --psk)      PSK=${2:?--psk needs a value}; shift 2 ;;
    --ip)       STATIC_IP=${2:?--ip needs a value}; IP_SET=1; MODE=static; shift 2 ;;
    --gateway)  GATEWAY=${2:?--gateway needs a value}; shift 2 ;;
    --dhcp)     MODE=dhcp; IP_SET=0; shift ;;
    --ifaces)   IFACES=${2:?--ifaces needs a value}; shift 2 ;;
    --nm)       NM_MODE=${2:?--nm needs a value}; shift 2 ;;
    --dhcpcd)   DHCPCD_MODE=${2:?--dhcpcd needs a value}; shift 2 ;;
    --dry-run)  DRYRUN=1; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *)          die "unknown option: $1 (try --help)" ;;
  esac
done

# --------------------------------------------------------------------------
# uninstall
# --------------------------------------------------------------------------
if [ "$UNINSTALL" -eq 1 ]; then
  [ "$(id -u)" -eq 0 ] || die "run as root"
  say "stopping and removing $SERVICE_NAME"
  run systemctl disable --now "$SERVICE_NAME" || true
  # The helper takes its address and its route back off the interface when it
  # stops. If it was not running, they can still be sitting there, and a
  # default route left on a downed interface swallows the replies to
  # everything else on the host.
  for i in $IFACES; do
    [ -e "/sys/class/net/$i" ] || continue
    run ip -4 addr flush dev "$i"
    run ip route flush dev "$i"
  done
  run rm -f "$UNIT" "$RUNTIME"
  run systemctl daemon-reload
  say "removing $WPA_CONF"
  run rm -f "$WPA_CONF"
  NM_CONF=/etc/NetworkManager/conf.d/99-wifi-autoconnect-unmanaged.conf
  if [ -f "$NM_CONF" ]; then
    say "restoring NetworkManager handling of the interfaces"
    run rm -f "$NM_CONF"
    run systemctl reload NetworkManager
  fi
  if [ -f "$DHCP_CONF" ] && grep -qF "$DHCPCD_BEGIN" "$DHCP_CONF"; then
    say "restoring $DHCP_CONF"
    run dhcpcd_restore_conf
    run dhcpcd_restart
  fi
  say "done. If NetworkManager was set to 'stop', re-enable it by hand."
  exit 0
fi

# --------------------------------------------------------------------------
# validate
# --------------------------------------------------------------------------
[ "$(id -u)" -eq 0 ] || die "run as root (needs apt + systemd)"

# --------------------------------------------------------------------------
# interactive input
#
# --ssid / --psk are optional when a terminal is attached; we prompt instead
# so credentials never end up in shell history or in `ps` output. With no
# terminal (CI, ansible) the flags stay mandatory.
# --------------------------------------------------------------------------
have_tty() { [ -t 0 ] && [ -t 2 ]; }

prompt_ssid() {
  local rc=0
  printf 'WiFi SSID              : ' >&2
  IFS= read -r SSID || rc=$?
  [ "$rc" -eq 0 ] || die "could not read the SSID from stdin"
}

prompt_psk() {
  local rc=0 hidden=""
  printf 'WiFi passphrase (8-63) : ' >&2
  if stty -echo 2>/dev/null; then
    hidden=1
    # Without this, a Ctrl-C here would leave the terminal stuck in no-echo.
    trap 'stty echo 2>/dev/null || true' EXIT INT TERM
  fi
  IFS= read -r PSK || rc=$?
  if [ -n "$hidden" ]; then
    stty echo 2>/dev/null || true
    trap - EXIT INT TERM
  fi
  printf '\n' >&2
  [ "$rc" -eq 0 ] || die "could not read the passphrase from stdin"
}

if [ -z "$SSID" ]; then
  if have_tty; then
    prompt_ssid
  else
    die "--ssid is required when there is no terminal (see --help)"
  fi
fi

if [ -z "$PSK" ]; then
  if have_tty; then
    prompt_psk
  else
    die "--psk is required when there is no terminal (see --help)"
  fi
fi

prompt_yes_no() {
  # $1 = question, $2 = default (y|n); returns 0 for yes, sets REPLY
  local q="$1" def="$2" rc=0 opts
  case "$def" in
    y) opts="Y/n" ;;
    *) opts="y/N" ;;
  esac
  printf '%s [%s] ' "$q" "$opts" >&2
  IFS= read -r REPLY || rc=$?
  [ "$rc" -eq 0 ] || die "could not read from stdin"
  if [ -z "$REPLY" ]; then REPLY=$def; fi
  case "$REPLY" in
    [Yy]|[Yy][Ee][Ss]) return 0 ;;
    *)                return 1 ;;
  esac
}

prompt_default() {
  # $1 = label, $2 = current value; echoes the value to use
  local label="$1" cur="$2" ans=""
  printf '%-25s: [%s] ' "$label" "$cur" >&2
  IFS= read -r ans || true
  [ -n "$ans" ] && printf '%s' "$ans" || printf '%s' "$cur"
}

if [ -z "$MODE" ]; then
  if have_tty; then
    if prompt_yes_no "use a static IP address?" y; then MODE=static; else MODE=dhcp; fi
    say "address mode: $MODE"
  else
    die "specify --ip ADDR/PREFIX for a static address or --dhcp (see --help)"
  fi
fi

if [ "$MODE" = static ] && [ "$IP_SET" -eq 0 ] && have_tty; then
  STATIC_IP=$(prompt_default "static address" "$STATIC_IP")
  GATEWAY=$(prompt_default "gateway" "$GATEWAY")
fi

if [ "$MODE" = dhcp ] && [ "$NM_MODE" = keep ]; then
  warn "--dhcp with --nm keep: NetworkManager may run its own DHCP client and"
  warn "  clash with ours. Use --nm unmanage (default) for DHCP mode."
fi

[ ${#PSK} -ge 8 ] && [ ${#PSK} -le 63 ] \
  || die "--psk must be 8-63 characters (got ${#PSK})"
case "$STATIC_IP" in */*) ;; *) STATIC_IP="$STATIC_IP/32" ;; esac
case "$NM_MODE" in unmanage|keep|stop) ;; *) die "--nm must be unmanage|keep|stop" ;; esac
case "$DHCPCD_MODE" in deny|keep) ;; *) die "--dhcpcd must be deny|keep" ;; esac

# --------------------------------------------------------------------------
# wpa_supplicant source
#
# /usr/bin/wpa_supplicant is the default target. The distro "wpasupplicant"
# package installs its binary under /usr/sbin (only /usr/bin on a usrmerge
# system), so both paths are tried, in that order. /usr/local/bin is the
# last resort: a build the operator maintains by hand.
#
# Presence is not enough. A package build linked against a mismatched libnl
# exists, is executable, and exits 127 — the exact failure this script
# exists to avoid. Every candidate is therefore executed with -v first.
# --------------------------------------------------------------------------
WSPA_PRIMARY=/usr/bin/wpa_supplicant
WSPA_PKG=/usr/sbin/wpa_supplicant
WSPA_LOCAL=/usr/local/bin/wpa_supplicant
WSPA=""
NEED_WSPA_PKG=0
REINSTALL=0

usable_wspa() { [ -n "${1:-}" ] && [ -x "$1" ] && "$1" -v >/dev/null 2>&1; }

for c in "$WSPA_PRIMARY" "$WSPA_PKG"; do
  if usable_wspa "$c"; then WSPA=$c; break; fi
done

if [ -n "$WSPA" ]; then
  say "using wpa_supplicant: $WSPA"
else
  if [ -x "$WSPA_PRIMARY" ] || [ -x "$WSPA_PKG" ]; then
    warn "a wpa_supplicant is installed but does not run; reinstalling the package"
    REINSTALL=1
  else
    say "no wpa_supplicant found; installing the distro wpasupplicant package"
  fi
  NEED_WSPA_PKG=1
fi

# --------------------------------------------------------------------------
# packages
# --------------------------------------------------------------------------
need_pkgs=()

# Add a package only once: wpasupplicant can be wanted by both the binary and
# the wpa_cli check, and "apt install foo foo" is noise at best.
add_pkg() {
  local want="$1" have
  for have in "${need_pkgs[@]:-}"; do
    [ "$have" = "$want" ] && return 0
  done
  need_pkgs+=("$want")
}

command -v ip        >/dev/null 2>&1 || add_pkg iproute2
command -v systemctl >/dev/null 2>&1 \
  || die "systemd is required; this script does not support non-systemd systems"

[ "$NEED_WSPA_PKG" -eq 0 ] || add_pkg wpasupplicant

# wpa_cli drives the control socket, so a broken one is as fatal as a
# missing one — the helper would sit there waiting for a status it can never
# read. It is therefore tested by running it, like wpa_supplicant.
usable_wpa_cli() { [ -n "${1:-}" ] && [ -x "$1" ] && "$1" -v >/dev/null 2>&1; }

WPA_CLI=""
if [ -n "$WSPA" ]; then
  for c in "$(dirname "$WSPA")/wpa_cli" /usr/bin/wpa_cli /usr/sbin/wpa_cli \
           "$(command -v wpa_cli 2>/dev/null || true)"; do
    if usable_wpa_cli "$c"; then WPA_CLI=$c; break; fi
  done
fi
if [ -z "$WPA_CLI" ]; then
  say "no working wpa_cli found; adding the distro package for it"
  add_pkg wpasupplicant
  REINSTALL=1
fi

if [ ${#need_pkgs[@]} -gt 0 ]; then
  say "installing packages: ${need_pkgs[*]}"
  run env DEBIAN_FRONTEND=noninteractive apt-get update -qq
  if [ "$REINSTALL" -eq 1 ]; then
    run env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --reinstall "${need_pkgs[@]}"
  else
    run env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "${need_pkgs[@]}"
  fi
fi

# Re-test after the install: the package may have just put it in place.
if [ -z "$WSPA" ]; then
  for c in "$WSPA_PRIMARY" "$WSPA_PKG" /sbin/wpa_supplicant; do
    if usable_wspa "$c"; then WSPA=$c; break; fi
  done
  [ -z "$WSPA" ] || say "using wpa_supplicant: $WSPA"
fi

# Last resort: the operator's own build, or anything else in PATH.
if [ -z "$WSPA" ]; then
  for c in "$WSPA_LOCAL" "$(command -v wpa_supplicant 2>/dev/null || true)"; do
    if usable_wspa "$c"; then
      WSPA=$c
      warn "the package build is unusable; falling back to $c"
      break
    fi
  done
fi

# Same again for wpa_cli now that the package had its chance.
if [ -z "$WPA_CLI" ] && [ -n "$WSPA" ]; then
  for c in "$(dirname "$WSPA")/wpa_cli" /usr/bin/wpa_cli /usr/sbin/wpa_cli \
           "$(command -v wpa_cli 2>/dev/null || true)"; do
    if usable_wpa_cli "$c"; then WPA_CLI=$c; break; fi
  done
fi

if [ -z "$WSPA" ]; then
  if [ "$DRYRUN" -eq 1 ]; then
    # Nothing is installed during a dry run, so the package would be the
    # source. Use its usual path so the rest of the script still works.
    say "dry run: wpa_supplicant would come from the wpasupplicant package"
    WSPA=$WSPA_PKG
    WPA_CLI=$(dirname "$WSPA_PKG")/wpa_cli
  else
    die "no working wpa_supplicant found (tried $WSPA_PRIMARY, $WSPA_PKG, $WSPA_LOCAL)"
  fi
else
  [ -n "$WPA_CLI" ] || die "no working wpa_cli found next to $WSPA"
fi

if [ "$DRYRUN" -eq 0 ]; then
  "$WSPA" -v 2>&1 | head -1 | sed 's/^/   /'
fi

# --------------------------------------------------------------------------
# DHCP client
#
# Only needed in dhcp mode: the helper shells out to a real client once the
# link is associated. The clients take different flags, so record which one
# was picked together with the argument that makes it bind one interface.
# --------------------------------------------------------------------------
DHCP_CLIENT=""
DHCP_ARGS=""
if [ "$MODE" = dhcp ]; then
  for c in /sbin/dhclient /usr/sbin/dhclient /sbin/udhcpc /usr/bin/udhcpc \
           /sbin/dhcpcd /usr/sbin/dhcpcd; do
    if [ -x "$c" ]; then DHCP_CLIENT=$c; break; fi
  done

  if [ -z "$DHCP_CLIENT" ]; then
    say "no DHCP client found; trying busybox (provides udhcpc)"
    run env DEBIAN_FRONTEND=noninteractive apt-get update -qq
    run env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq busybox || true
    for c in /bin/udhcpc /usr/bin/udhcpc /sbin/dhclient /usr/sbin/dhclient; do
      [ -x "$c" ] && { DHCP_CLIENT=$c; break; }
    done
  fi

  if [ -z "$DHCP_CLIENT" ]; then
    if [ "$DRYRUN" -eq 1 ]; then
      # The client would have been installed by the step above. A dry run
      # must not die over something it never got the chance to do.
      say "dry run: a DHCP client would be installed"
      DHCP_CLIENT=/sbin/dhclient
    else
      die "no usable DHCP client found (need dhclient or udhcpc)"
    fi
  fi

  case "$(basename "$DHCP_CLIENT")" in
    udhcpc)  DHCP_ARGS="-i" ;;   # udhcpc -i IFACE
    dhclient) DHCP_ARGS="-1" ;;  # dhclient -1 IFACE: one attempt, foreground
    dhcpcd)  DHCP_ARGS="" ;;    # dhcpcd binds the interface by name
    *)       DHCP_ARGS="" ;;
  esac
  say "dhcp client: $DHCP_CLIENT $DHCP_ARGS"
fi

# --------------------------------------------------------------------------
# NetworkManager coexistence
# --------------------------------------------------------------------------
if [ "$NM_MODE" != "keep" ] && systemctl list-unit-files 2>/dev/null | grep -q '^NetworkManager.service'; then
  case "$NM_MODE" in
    stop)
      say "stopping and disabling NetworkManager"
      run systemctl disable --now NetworkManager.service
      ;;
    unmanage)
      # "nmcli device set ... managed no" is runtime-only and is lost on the
      # next boot. A keyfile under conf.d is the mechanism that persists.
      say "marking target interfaces unmanaged in NetworkManager"
      NM_DEVICES=""
      for i in $IFACES; do
        NM_DEVICES="${NM_DEVICES:+${NM_DEVICES};}interface-name:${i}"
      done
      NM_CONF=/etc/NetworkManager/conf.d/99-wifi-autoconnect-unmanaged.conf
      if [ "$DRYRUN" -eq 1 ]; then
        printf '   would write %s:\n' "$NM_CONF"
        printf '[keyfile]\nunmanaged-devices=%s\n' "$NM_DEVICES" | sed 's/^/     /'
      else
        mkdir -p /etc/NetworkManager/conf.d
        printf '[keyfile]\nunmanaged-devices=%s\n' "$NM_DEVICES" > "$NM_CONF"
        systemctl reload NetworkManager 2>/dev/null \
          || systemctl restart NetworkManager 2>/dev/null || true
        say "wrote $NM_CONF"
      fi
      ;;
  esac
fi

if [ "$DHCPCD_MODE" = deny ] && dhcpcd_present; then
  DHCPCD_LIST=$(dhcpcd_denied_list)
  say "denying dhcpcd on: $DHCPCD_LIST"
  if [ "$DRYRUN" -eq 1 ]; then
    printf '   would rewrite %s with a denyinterfaces line\n' "$DHCP_CONF"
    printf '   would restart dhcpcd so the rule takes effect\n'
  else
    [ -f "$DHCP_CONF" ] || : >"$DHCP_CONF"
    dhcpcd_write_conf "$DHCPCD_LIST" || die "could not update $DHCP_CONF"
    say "wrote $DHCP_CONF"
    # Restarting dhcpcd makes the rule effective now. The wifi service is
    # restarted further down; its helper flushes the interface, which clears
    # the address and routes the old dhcpcd left behind.
    dhcpcd_restart || warn "could not restart dhcpcd; the rule applies from the next start"
  fi
fi

# --------------------------------------------------------------------------
# wpa_supplicant config
# --------------------------------------------------------------------------
say "writing $WPA_CONF"
if [ "$DRYRUN" -eq 1 ]; then
  printf '   would write ssid=%s psk=<redacted> into %s\n' "$SSID" "$WPA_CONF"
else
  mkdir -p /etc/wpa_supplicant
  cat >"$WPA_CONF" <<EOF
# Generated by setup-wifi-autoconnect.sh
ctrl_interface=/var/run/wpa_supplicant
update_config=1
country=US

network={
	ssid="$SSID"
	psk="$PSK"
	key_mgmt=WPA-PSK
}
EOF
  chmod 600 "$WPA_CONF"
fi

# --------------------------------------------------------------------------
# runtime helper
# --------------------------------------------------------------------------
say "writing $RUNTIME"
if [ "$DRYRUN" -eq 1 ]; then
  printf '   would write the runtime helper\n'
else
  # The helper is emitted in three pieces: shared preamble, the address block
  # for this mode only, then a shared tail. Deciding the branch here rather
  # than at run time keeps every generated file free of code that could never
  # execute. The mode string below is expanded now, so it lands as a literal.
  cat >"$RUNTIME" <<EOF
#!/bin/sh
# Generated by setup-wifi-autoconnect.sh — do not edit by hand.
# Picks the first available interface from IFACES, forces the rest down,
# associates with "$SSID", then sets the address (mode $MODE).
set -u

IFACES="$IFACES"
CONF="$WPA_CONF"
CTRL=/var/run/wpa_supplicant
WSPA="$WSPA"
WPA_CLI="$WPA_CLI"

log() { echo "wifi-autoconnect: \$*"; }

# 1. wait for an interface (first in the list wins)
IF=""
while [ -z "\$IF" ]; do
  for i in \$IFACES; do
    if [ -e "/sys/class/net/\$i" ]; then IF="\$i"; break; fi
  done
  [ -n "\$IF" ] || sleep 2
done

# 2. every other candidate stays down, so the address cannot collide
for i in \$IFACES; do
  [ "\$i" = "\$IF" ] && continue
  [ -e "/sys/class/net/\$i" ] && ip link set "\$i" down 2>/dev/null
done

ip link set "\$IF" up
log "using interface \$IF (mode $MODE)"
EOF

  # 3. address assignment — only the branch this mode actually runs
  if [ "$MODE" = static ]; then
    cat >>"$RUNTIME" <<EOF

STATIC_IP="$STATIC_IP"
GATEWAY="$GATEWAY"

# 3. apply the static address once associated
(
  while :; do
    if [ -e "\$CTRL/\$IF" ] && \\
       "\$WPA_CLI" -p "\$CTRL" -i "\$IF" status 2>/dev/null | grep -q "wpa_state=COMPLETED"; then
      ip addr flush dev "\$IF" 2>/dev/null
      ip addr replace \$STATIC_IP dev "\$IF"
      ip route replace default via \$GATEWAY dev "\$IF"
      log "associated; static \$STATIC_IP applied to \$IF"
      break
    fi
    sleep 3
  done
) &
EOF
  else
    cat >>"$RUNTIME" <<EOF

DHCP_CLIENT="$DHCP_CLIENT"
DHCP_ARGS="$DHCP_ARGS"

# 3. request a lease once associated
(
  while :; do
    if [ -e "\$CTRL/\$IF" ] && \\
       "\$WPA_CLI" -p "\$CTRL" -i "\$IF" status 2>/dev/null | grep -q "wpa_state=COMPLETED"; then
      log "associated; requesting a lease with \$DHCP_CLIENT"
      "\$DHCP_CLIENT" \$DHCP_ARGS "\$IF" || log "\$DHCP_CLIENT returned non-zero"
      break
    fi
    sleep 3
  done
) &
EOF
  fi

  # 4. run wpa_supplicant and clean up after it
  #
  # wpa_supplicant is started as a child rather than exec'd, so that this
  # script is still around to take the address and the route back off the
  # interface when it is stopped. Leaving them behind is not harmless: a
  # default route through an interface that is down is skipped by the kernel
  # when it looks up a destination, but it still wins the lookup, so the
  # replies to everything else go out of a dead link and are lost.
  cat >>"$RUNTIME" <<EOF

cleanup() {
  log "releasing the address and the route on \$IF"
  if [ "\$MODE" = static ]; then
    ip -4 addr del \$STATIC_IP dev "\$IF" 2>/dev/null
  fi
  ip route del default via \$GATEWAY dev "\$IF" 2>/dev/null
}
trap 'cleanup' EXIT
trap 'exit 0' TERM INT HUP

# 5. run wpa_supplicant in the foreground so systemd can supervise it
"\$WSPA" -i "\$IF" -c "\$CONF" &
WSPA_PID=\$!
wait "\$WSPA_PID"
EOF
  chmod 755 "$RUNTIME"
fi

# --------------------------------------------------------------------------
# systemd unit
# --------------------------------------------------------------------------
say "writing $UNIT"
if [ "$DRYRUN" -eq 1 ]; then
  printf '   would write the systemd unit\n'
else
  if [ "$MODE" = static ]; then
    UNIT_DESC="WiFi autoconnect to \"$SSID\" (static $STATIC_IP)"
  else
    UNIT_DESC="WiFi autoconnect to \"$SSID\" (DHCP)"
  fi
  cat >"$UNIT" <<EOF
[Unit]
Description=$UNIT_DESC
Wants=network-pre.target
Before=network.target network-online.target
After=systemd-udevd.service

[Service]
Type=simple
ExecStart=$RUNTIME
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
fi

# The distro's wpa_supplicant.service would start a second instance on the
# same interfaces; ours owns them. Only disable it if it is actually there.
if [ -f /lib/systemd/system/wpa_supplicant.service ] && [ "$DRYRUN" -eq 0 ]; then
  say "disabling distro wpa_supplicant.service (avoids a second instance)"
  systemctl disable wpa_supplicant.service >/dev/null 2>&1 || true
fi

if [ "$DRYRUN" -eq 0 ]; then
  say "enabling and starting $SERVICE_NAME"
  systemctl enable "$SERVICE_NAME" >/dev/null
  systemctl restart "$SERVICE_NAME"

  say "waiting for association (up to 45s)"
  ok=0
  used=""
  for _ in $(seq 1 45); do
    # The helper may have picked any interface in the list, so check them all
    # rather than assuming the first entry won.
    for i in $IFACES; do
      if "$WPA_CLI" -p /var/run/wpa_supplicant -i "$i" status 2>/dev/null \
          | grep -q "wpa_state=COMPLETED"; then
        ok=1; used=$i; break
      fi
    done
    [ "$ok" -eq 1 ] && break
    sleep 1
  done

  if [ "$ok" -eq 1 ]; then
    printf '\n\033[1;32mOK\033[0m associated on %s.\n' "$used"
    # The helper applies the address a moment after the association, so
    # wait for it before reporting: otherwise the listing below still shows
    # whatever the interface had before.
    if [ "$MODE" = dhcp ]; then
      say "waiting up to 20s for a DHCP lease"
      for _ in $(seq 1 20); do
        if ip -4 addr show "$used" 2>/dev/null | grep -q 'inet '; then break; fi
        sleep 1
      done
    else
      say "waiting up to 20s for the static address"
      for _ in $(seq 1 20); do
        if ip -4 addr show "$used" 2>/dev/null | grep -qF "inet $STATIC_IP"; then break; fi
        sleep 1
      done
    fi
    printf 'current addresses:\n'
    ip -4 -br addr
    ip route | head -3
  else
    warn "no association yet. Check: journalctl -u $SERVICE_NAME -f"
  fi
else
  say "dry run complete; nothing was changed"
fi

cat <<EOF

To remove everything this script created:
  sudo $0 --uninstall
EOF