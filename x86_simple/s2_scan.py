#!/usr/bin/env python3
"""S2 serial 第一圈掃描 — pyrplidarsdk 版 (x86 / ARM64 共用)。
會讓雷達轉起來，取 N 圈點雲並存 csv。

安裝: pip install pyrplidarsdk  (x86_64 與 aarch64 皆有 manylinux wheel，已驗證)
用法:
  python3 s2_scan.py [--port /dev/ttyUSB0] [--scans 5] [--out scan.csv]
"""
import argparse
import sys
import time

try:
    import pyrplidarsdk
except ImportError:
    print("ERROR: 缺 pyrplidarsdk，請先裝: pip install pyrplidarsdk")
    sys.exit(1)


def main() -> int:
    ap = argparse.ArgumentParser(description="RPLIDAR S2 第一圈掃描 (pyrplidarsdk)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000, help="S2 serial 固定 1000000")
    ap.add_argument("--scans", type=int, default=5)
    ap.add_argument("--out", default="scan.csv")
    args = ap.parse_args()

    driver = pyrplidarsdk.RplidarDriver(port=args.port, baudrate=args.baud)
    if not driver.connect():
        print(f"FAIL: connect({args.port}@{args.baud}) 失敗（被佔用？權限？線？）")
        return 1
    try:
        info = driver.get_device_info()
        print(f"device_info: {info}")
        health = driver.get_health()
        print(f"health: {health}")
        if not driver.start_scan():
            print("FAIL: start_scan 回傳 False（檢查供電/串口佔用）")
            return 1
        print("scanning... (雷達應已轉起)")
        total = 0
        with open(args.out, "w") as f:
            f.write("scan,angle_deg,range_m,quality\n")
            for i in range(args.scans):
                time.sleep(0.5)
                data = driver.get_scan_data()
                if not data:
                    print(f"  scan {i + 1}: <no data>")
                    continue
                angles, ranges, qualities = data
                n = len(angles)
                total += n
                print(f"  scan {i + 1}: {n} points, "
                      f"angle[{min(angles):.1f},{max(angles):.1f}] "
                      f"range[{min(ranges):.2f},{max(ranges):.2f}]m")
                for a, r, q in zip(angles, ranges, qualities):
                    f.write(f"{i},{a:.2f},{r:.3f},{q}\n")
        print(f"DONE: 共 {total} 點 -> {args.out}")
        if total == 0:
            print("FAIL: 有轉無點 — 檢查遮擋/供電/光窗")
            return 1
        return 0
    finally:
        try:
            driver.stop_scan()
        except Exception:
            pass
        try:
            driver.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
