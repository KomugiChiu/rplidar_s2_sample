#!/usr/bin/env python3
"""S2 serial 最簡健康檢查 — 零額外依賴 (只要 pyserial, Ubuntu 24.04 內建 python3-serial)。
x86 / ARM64 共用。只送 GET_INFO + GET_HEALTH，不起轉，不會讓雷達轉起來。

用法:
  python3 s2_info_check.py [--port /dev/ttyUSB0] [--baud 1000000]
回傳值: 0=健康, 1=異常
實測: S2 serial, CP2102N, 1000000 baud -> INFO type 0x04 / HEALTH OK(0)
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: 缺 pyserial，請先裝: sudo apt install python3-serial  # 或 pip install pyserial")
    sys.exit(1)


def transact(ser, cmd: bytes, wait: float = 0.3):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(cmd)
    time.sleep(wait)
    desc = ser.read(7)
    if len(desc) != 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
        return None, desc
    length = struct.unpack("<I", desc[2:6])[0] & 0x3FFFFFFF
    dtype = desc[6]
    payload = ser.read(length)
    return (dtype, payload), desc


def main() -> int:
    ap = argparse.ArgumentParser(description="RPLIDAR S2 serial 健康檢查 (pyserial only)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000,
                    help="S2 serial 固定 1000000，用錯 (115200/256000) 會無回應")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except Exception as e:
        print(f"ERROR: 開串口失敗 {args.port}@{args.baud}: {e}")
        print("排查: ls -l /dev/ttyUSB*; id | grep dialout; ModemManager 是否佔用")
        return 1

    ok = True
    with ser:
        # 先停掉可能殘留的掃描 (前一次 s2_scan 未正常 stop 時會續轉) 再問
        ser.write(bytes([0xA5, 0x25]))  # STOP
        time.sleep(0.5)
        ser.reset_input_buffer()
        # 1. GET_INFO: A5 50
        res, raw = transact(ser, bytes([0xA5, 0x50]))
        if res is None:
            print(f"FAIL: GET_INFO 無回應 (raw={raw.hex(' ') if raw else '<empty>'})")
            print("排查: baud 必須是 1000000；檢查供電 5V；換短線直插主機 USB 口")
            ok = False
        else:
            dtype, payload = res
            if dtype != 0x04 or len(payload) < 4:
                print(f"FAIL: INFO 格式異常 type=0x{dtype:02X} len={len(payload)}")
                ok = False
            else:
                model, fw_min, fw_maj, hw = payload[0], payload[1], payload[2], payload[3]
                sn = payload[4:].hex() if len(payload) > 4 else ""
                print(f"INFO OK: model={model} fw={fw_maj}.{fw_min} hw={hw} sn={sn}")
                print(f"  raw={payload.hex(' ')}")

        # 2. GET_HEALTH: A5 52
        res, raw = transact(ser, bytes([0xA5, 0x52]))
        if res is None:
            print(f"FAIL: GET_HEALTH 無回應 (raw={raw.hex(' ') if raw else '<empty>'})")
            ok = False
        else:
            dtype, payload = res
            if len(payload) < 3:
                print(f"FAIL: HEALTH 格式異常 len={len(payload)}")
                ok = False
            else:
                status = {0: "OK", 1: "WARN", 2: "ERROR"}.get(payload[0], str(payload[0]))
                err = struct.unpack("<H", payload[1:3])[0]
                print(f"HEALTH {status}: status={payload[0]} error_code={err}")
                if payload[0] != 0:
                    ok = False

    print("RESULT: PASS — 雷達活著且健康" if ok else "RESULT: FAIL — 見上方排查")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
