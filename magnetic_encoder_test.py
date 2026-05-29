from gpiozero import DigitalInputDevice
import time
import csv
import threading
import math

# --- 設定  ---
SENSOR_PIN = 4             # 緑のケーブルのGPIOピン
MAGNET_PAIRS = 2
FILENAME = 'peak_measurements.csv'

# --- ケーブル長の計算　　---
WHEEL_DIAMETER_MM = 32     # ホイールの直径（mm）
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM  # 1回転あたり約100.53 mm

# --- 初期化 ---
sensor = DigitalInputDevice(SENSOR_PIN, pull_up=True, bounce_time=0.005)  # 5msのデバウンス（チャタリング防止)
pulse_count = 0
start_time = time.monotonic()   
lock = threading.Lock()       

# これらの変数はメインブロックで割り当てられます
writer = None
csv_file = None

# --- 割り込み関数 ---
def detect_pulse():
    global pulse_count, writer

    with lock:
        pulse_count += 1
        revolutions = pulse_count / MAGNET_PAIRS

    elapsed_time = round(time.monotonic() - start_time, 3)
    length_mm = round((revolutions - (1 / MAGNET_PAIRS)) * WHEEL_CIRCUMFERENCE_MM, 2)

    if writer is not None:
        # [時間, 状態(0V), 回転数, 長さ(mm)] を書き込みます
        writer.writerow([elapsed_time, 0, revolutions, length_mm])
        # グラフを綺麗にするため、直後に立ち上がり（3.3V）をシミュレートします
        writer.writerow([elapsed_time + 0.001, 3.3, revolutions, length_mm])
        csv_file.flush()

    print(f"[{elapsed_time}s] パルス検出！ -> 回転数: {revolutions} | ケーブル長: {length_mm} mm")


print("オドメーターシステムが有効になりました（軽量メソッド）")
print("ケーブルを引き出すためにホイールを回してください！")

try:
    # 割り込みを有効にする前にファイルを開きます
    with open(FILENAME, mode='w', newline='') as f:
        csv_file = f
        writer = csv.writer(csv_file)
        
        # CSVのヘッダーも英語に変更しています
        writer.writerow(['Time (s)', 'Voltage (V)', 'Cumulative Revolutions', 'Cable Length (mm)'])
        
        # 初期状態（3.3V）を書き込みます
        writer.writerow([0, 3.3, 0, 0])

        # ファイルの準備ができたので、割り込みをバインドします
        sensor.when_deactivated = detect_pulse

        # メインプログラムは何もしません（CPU使用率ゼロ）
        # Ctrl+Cが押されるのを待つだけです
        while True:
            time.sleep(1)

except KeyboardInterrupt:
    print(f"\n終了しました。{FILENAME} にすべてのパルスデータが保存されています！")
    with lock:
        total_revolutions = pulse_count / MAGNET_PAIRS
        print(f"記録された合計: {total_revolutions} 回転")
        print(f"引き出されたケーブルの長さ: {round(total_revolutions * WHEEL_CIRCUMFERENCE_MM, 2)} mm")
