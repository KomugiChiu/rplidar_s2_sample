# ARM64 簡單驗證版 (Ubuntu 24.04 + Jazzy 板子用)

與 `x86_simple/` 同內容，純 Python，不用 ROS，不用編譯，板子上有網路就能跑。

## 板上步驟 (複製本資料夾過去即可)

```bash
# 0. 接線：S2 serial -> USB，確認裝置出現
ls -l /dev/ttyUSB*; lsusb -d 10c4:ea60

# 1. 零依賴健康檢查 (pyserial，Ubuntu 預裝 python3-serial)
python3 s2_info_check.py --port /dev/ttyUSB0
# 期待: INFO OK + HEALTH OK + RESULT: PASS

# 2. 第一圈掃描 (需 pip，一次就好)
pip install -r requirements.txt
python3 s2_scan.py --port /dev/ttyUSB0 --scans 5 --out scan.csv
# 期待: 每圈數百~數千點，scan.csv 有資料；雷達會轉，屬正常
```

## ARM64 注意 (與 x86 差異)

* `pip install pyrplidarsdk` 在 aarch64 有 manylinux wheel (已確認 0.1.1 有 aarch64 版)，不用編譯。
* Python 需 >=3.10；Ubuntu 24.04 預設 3.12 可直接用。
* 供電比 x86 更敏感：板載 USB 電流小，務必用帶電源 hub 或外供 5V，否則轉速上不去、點數忽多忽少。
* 若 `ModemManager` 佔用串口：`sudo systemctl disable --now ModemManager`。
* baud 固定 `1000000`，用 115200/256000 會無回應，不是壞掉。
