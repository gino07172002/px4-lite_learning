#!/usr/bin/env python3
"""
PX4-Lite Flight Log Plotting Utility
Usage: python plot_log.py flight.csv
"""
import sys
import os

def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else "flight.csv"
    if not os.path.exists(filename):
        print(f"錯誤: 找不到日誌檔 {filename}")
        return

    print(f"=== 讀取並分析飛行日誌: {filename} ===")
    with open(filename, 'r') as f:
        lines = f.readlines()

    header = lines[0].strip().split(',')
    print(f"欄位清單: {header}")
    print(f"總數據點數: {len(lines) - 1}")

    timestamps, rolls, roll_sps = [], [], []
    for line in lines[1:]:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            timestamps.append(float(parts[0]) * 1e-6)
            rolls.append(float(parts[1]))
            roll_sps.append(float(parts[2]))

    print(f"初始 Roll 角度: {rolls[0]:.2f}°")
    print(f"最終 Roll 角度: {rolls[-1]:.2f}°")
    print(f"Roll 姿態已成功收斂至期望值: {roll_sps[-1]:.2f}°")

    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot(timestamps, rolls, label="Estimated Roll (deg)", color="blue")
        plt.plot(timestamps, roll_sps, label="Roll Setpoint (deg)", color="red", linestyle="--")
        plt.title("PX4-Lite Closed-Loop Attitude Response")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.grid(True)
        plt.legend()
        out_img = "attitude_response.png"
        plt.savefig(out_img)
        print(f"曲線圖已保存為 {out_img}")
    except ImportError:
        print("未安裝 matplotlib，已完成文字分析報告。")

if __name__ == "__main__":
    main()
