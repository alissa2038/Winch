#!/usr/bin/env python3
"""
ウインチ制御 + LiDAR緊急停止
起動時にモードを選択:
  [m] 手動モード : ↑↓で動く / q(+10us) a(-10us) で速度調整（動作中・停止中どちらでも可）
  [a] 自動モード : 方向・速度を入力 → 自動回転 → レーザーで停止（巻き取りのみ）

共通:
  スペース      : 終了
  r             : 緊急停止解除
  LiDAR差 DIFF_THRESHOLD 以上 : 緊急停止
  LiDAR異常値                 : 緊急停止
"""

import sys
import tty
import termios
import pigpio
import threading
import time
import smbus
import select as _select
import math
import random

# =====================
#   ウインチ設定 (Constantes globales)
# =====================
PIN                = 18
STOP_fwd           = 1440
STOP_center        = 1400
STOP_rev           = 1370
FWD_DEFAULT        = 1455   # 正転デフォルト（巻き取り）
REV_DEFAULT        = 1345   # 逆転デフォルト（繰り出し）
PWM_FWD_MIN        = 1455   # 正転の最小値
PWM_FWD_MAX        = 2000   # 正転の最大値
PWM_REV_MIN        = 1000   # 逆転の最小値
PWM_REV_MAX        = 1345   # 逆転の最大値
SPEED_STEP         = 10     # q/a で変化する速度ステップ (us)

# =====================
#   ケーブル設定
# =====================
WINCH_RADIUS = 0.10   # ウインチの半径　10 cm
CABLE_LENGTH_MAX = 150.0    # 全体のケーブル長

# =====================
#   センサー設定
# =====================
LIDAR_ADDRESS  = 0x66  # LiDARセンサーのアドレス
SENSOR_HZ      = 10    # 測定頻度 (回/秒)
DIFF_THRESHOLD = 10    # この差(cm)以上で緊急停止
VALID_MIN      = 2     # これ以下は異常値(cm)
VALID_MAX      = 800   # これ以上は異常値(cm)


class WinchController:
    def __init__(self):
        # =====================
        #   pigpio 初期化  PWM用ライブラリ
        # =====================
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("pigpiod が起動していません: sudo pigpiod")
            sys.exit()

        # =====================
        #   状態変数 
        # =====================
        self.fd = sys.stdin.fileno()  # ターミナル関係
        self.old = termios.tcgetattr(self.fd)
    
        self.running = True
        self.current_distance = None
        self.emergency = False   # 緊急停止中フラグ

        self.fwd_speed = FWD_DEFAULT
        self.rev_speed = REV_DEFAULT
        self.current_direction = None   # 'fwd' / 'rev' / None
        self.paused_direction = None
        self.msg_timer = None

        self.target_pwm = STOP_center  # 目標の速度
        self.current_pwm = STOP_center  # 現在のモーターの速度

        self.sensor_connected = False
        self.num_rotations = 0.0   # ウインチの合計回転数

        self.dropout_end_time = 0   # 震度シミュレーション
        self.next_dropout_time = time.time() + 10.0  # 最初の途切れは10秒後

        # スレッドの開始 
        threading.Thread(target=self.sensor_loop, daemon=True).start()
        threading.Thread(target=self.motor_loop, daemon=True).start()
        

        
        tty.setraw(self.fd)
        self.pi.set_servo_pulsewidth(PIN, STOP_center)

    # =====================
    #   緊急停止
    # =====================
    def emergency_stop(self, reason):
        if self.emergency:
            return   # すでに停止中なら何もしない
        self.emergency = True    # フラグON → キー入力を無視・センサーチェックも無視

        self.target_pwm = STOP_center    #STOPに戻しておく
        self.current_pwm = STOP_center

        self.current_direction = None
        self.paused_direction = None

        self.pi.set_servo_pulsewidth(PIN, STOP_center)
        print(f"\r\033[K  🛑 緊急停止: {reason}"           , flush=True)
        print(f"\r\n  解除するには r を押してください\r\n", flush=True)

    # =====================
    #   センサースレッド
    # =====================
    def sensor_loop(self):
        try:
            bus = smbus.SMBus(1)      #3と５のピンはI2Cバス1として機能するGPIOピン
        except Exception:
            return                    #I2Cがそもそも機能しない場合

        while self.running:
            try:
                raw  = bus.read_word_data(LIDAR_ADDRESS, 0)
                dist = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)    #バイト順が逆なので上下を入れ替える

                if not self.sensor_connected:
                    self.sensor_connected = True

                # 緊急停止中はチェックしない
                if self.emergency:
                    time.sleep(1.0 / SENSOR_HZ)
                    continue

                # 異常値チェック
                if dist < VALID_MIN or dist > VALID_MAX:
                    self.emergency_stop(f"異常値 {dist}cm")
                    time.sleep(1.0 / SENSOR_HZ)
                    continue

                # 差チェック
                if self.current_distance is not None:    #値が存在するか確認
                    diff = abs(dist - self.current_distance)
                    if diff >= DIFF_THRESHOLD:
                        self.emergency_stop(f"距離変化 {diff}cm ({self.current_distance} → {dist}cm)")
                        self.current_distance = None            #ソナーが通り過ぎたときに再び緊急停止しないように
                        continue

                self.current_distance = dist

            except Exception:
                if self.sensor_connected:
                    self.sensor_connected = False
                self.current_distance = None

            time.sleep(1.0 / SENSOR_HZ)

    # =====================
    #   モータースレッド（滑らかな加減速）
    # =====================
    def motor_loop(self):
        while self.running:
            # 緊急停止の時は、滑らかさ無視で即座にピタッと止める
            if self.emergency:
                self.current_pwm = STOP_center
                self.pi.set_servo_pulsewidth(PIN, STOP_center)
                time.sleep(0.1)
                continue
            
            # 目標と現在の差をチェックして、5、または20ずつ近づける
            if self.target_pwm in [STOP_center, STOP_fwd, STOP_rev]:
                step = 15  #素早く停止
            else:
                step = 5   #ゆっくり加速
            
            if self.current_pwm < self.target_pwm:
                # 正転へ向かう時、不感帯(1370〜1440未満)にいたら currentpwm = 1440 
                if self.target_pwm >= STOP_fwd and (STOP_rev <= self.current_pwm < STOP_fwd):
                    self.current_pwm = STOP_fwd
                else:
                    self.current_pwm = min(self.current_pwm + step, self.target_pwm)
                self.pi.set_servo_pulsewidth(PIN, self.current_pwm)

            elif self.current_pwm > self.target_pwm:
                # 逆転へ向かう時、不感帯(1370より大きく1440以下)にいたら 1370 まで飛ぶ
                if self.target_pwm <= STOP_rev and (STOP_rev < self.current_pwm <= STOP_fwd):
                    self.current_pwm = STOP_rev
                else:
                    self.current_pwm = max(self.current_pwm - step, self.target_pwm)
                self.pi.set_servo_pulsewidth(PIN, self.current_pwm)

            if self.current_pwm > STOP_fwd:
                self.num_rotations -= (ESTIMATED_ROT_PER_SEC * 0.02)  #繰り出し＝回転数減る
            elif self.current_pwm < STOP_rev:
                self.num_rotations += (ESTIMATED_ROT_PER_SEC * 0.02)  
                
            if self.num_rotations < 0:   # マイナスにはならないようにする
                self.num_rotations = 0.0
            
            # 0.02秒待つ（1秒間に50回更新 ＝ 滑らか)
            time.sleep(0.02)

    # =====================
    #   キー入力
    # =====================
    def get_key(self):
        #escを押してしまった場合、好きなキーを一度押したらリセットされます
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)   #'\x1b'に続きがある前提でreadする
            if ch2 == '[':            #続きが　[　でない場合、押されたのはesc
                ch3 = sys.stdin.read(1)
                return ch + ch2 + ch3
            else:
                return ch + ch2   #escを押してしまった時用 (esc=\x1b)
        return ch

    # =====================
    #   画面の再描画（3秒後に呼ばれる）
    # =====================
    def redraw_status(self):
        # \033[K で行を綺麗に掃除してから、いまの状態を表示し直す
        if self.paused_direction is not None:
            print(f"\r\033[K  ⏸ 一時停止 (スペースで再開) ", end='', flush=True)
        else:
            print(f"\r\033[K  ■ 停止 (方向キーを押してください) ", end='', flush=True)

    # =====================
    #   速度変更（停止中・動作中どちらでも可）
    # =====================
    def change_speed(self, delta):
        if self.current_direction == 'fwd':
            # 巻き取り中 → fwd_speedだけ変更
            self.fwd_speed = max(PWM_FWD_MIN, min(PWM_FWD_MAX, self.fwd_speed + delta))
            self.target_pwm = self.fwd_speed
            print(f"\r\033[K  ▲ 巻き取り中 ({self.fwd_speed}us) ", end='', flush=True)

        elif self.current_direction == 'rev':
            # 繰り出し中 → rev_speedだけ変更
            self.rev_speed = max(PWM_REV_MIN, min(PWM_REV_MAX, self.rev_speed - delta))
            self.target_pwm = self.rev_speed
            print(f"\r\033[K  ▼ 繰り出し中 ({self.rev_speed}us) ", end='', flush=True)

        else:
            # 停止中 → 両方の設定値を変更
            self.fwd_speed = max(PWM_FWD_MIN, min(PWM_FWD_MAX, self.fwd_speed + delta))
            self.rev_speed = max(PWM_REV_MIN, min(PWM_REV_MAX, self.rev_speed - delta))
            # 画面に速度変更のメッセージを表示
            print(f"\r\033[K  速度設定 → 巻き取り: {self.fwd_speed}us  繰り出し: {self.rev_speed}us ", end='', flush=True)

            # もし既にタイマーが動いていたら（連打した時）一旦キャンセル
            if self.msg_timer:
                self.msg_timer.cancel()
            
            # 1秒後に redraw_status を実行するタイマーをスタート！
            self.msg_timer = threading.Timer(1.0, self.redraw_status)
            self.msg_timer.start()


    # =====================
    #   手動モード
    # =====================
    def manual_mode(self):
        print(f"\r\n\n\n\n\n ---- 手動モード ---- \r\n")
        print(f"\r   現在の速度 → 巻き取り: {self.fwd_speed}us  繰り出し: {self.rev_speed}us\r\n")   
        print(f"\r  ┌────────────────── 手動モード ───────────────────┐\r\n")
        print(f"\r  │  【↑】        巻き取り   【↓】 繰り出し         │\r\n")
        print(f"\r  │  【スペース】 一時停止                          │\r\n")
        print(f"\r  │  【q / a】    速度変更                          │\r\n")
        print(f"\r  │  【b】        戻る                              │\r\n")
        print(f"\r  │  【s】        終了                              │\r\n")
        print(f"\r  └─────────────────────────────────────────────────┘\r\n")
        print(f"\r\n  ■ 停止 (方向キーを押して制御スタート) ", end='', flush=True)

        while True:
            key = self.get_key()

            if key == 's':
                self.target_pwm = STOP_center
                self.current_direction = None
                return 'quit'
            
            elif key == 'b':  #メニューに戻る
                self.target_pwm = STOP_center
                self.current_direction = None
                return 'menu'
            
            elif key == ' ':  # 一時停止
                if self.current_direction is not None:
                    # ①　ウインチ動いてる時 ->　一時停止して今の方向を記憶
                    self.paused_direction = self.current_direction
                    self.target_pwm = STOP_center
                    self.current_direction = None
                    print(f"\r\033[K  ⏸ 一時停止 (スペースで再開) ", end='', flush=True)
                else:
                    # ② 止まっている時 -> 記憶をもとに再開
                    if self.paused_direction == 'fwd':
                        self.current_direction = 'fwd'
                        self.target_pwm = self.fwd_speed
                        self.paused_direction = None  # 記憶をリセット
                        print(f"\r\033[K  ▲ 巻き取り中 ({self.fwd_speed}us) ", end='', flush=True)
                    elif self.paused_direction == 'rev':
                        self.current_direction = 'rev'
                        self.target_pwm = self.rev_speed
                        self.paused_direction = None  # 記憶をリセット
                        print(f"\r\033[K  ▼ 繰り出し中 ({self.rev_speed}us) ", end='', flush=True)
                    else:
                        # 記憶がない（起動直後など）場合は動かさない
                        print(f"\r  ■ 停止 (方向キーを押して制御スタート) ", end='', flush=True)
                
            # 緊急停止中は r 以外のキーをすべて無視
            if self.emergency:
                if key == 'r':
                    self.emergency = False
                    print(f"\r  ✅ 操作を再開できます\r\n\n", flush=True)
                    self.redraw_status()
                continue

            if key == '\x1b[A':   # ↑ 巻き取り開始
                self.current_direction = 'fwd'
                self.paused_direction = None   # 新しく矢印を押したら記憶はリセット
                self.target_pwm = self.fwd_speed
                print(f"\r\033[K  ▲ 巻き取り中 ({self.fwd_speed}us) ", end='', flush=True)

            elif key == '\x1b[B':   # ↓ 繰り出し開始
                self.current_direction = 'rev'
                self.paused_direction = None   # 新しく矢印を押したら記憶はリセット
                self.target_pwm = self.rev_speed
                print(f"\r\033[K  ▼ 繰り出し中 ({self.rev_speed}us) ", end='', flush=True)

            elif key == 'q':
                self.change_speed(+SPEED_STEP)

            elif key == 'a':
                self.change_speed(-SPEED_STEP)

    # =====================
    #   メインループ
    # =====================
    def run(self):
        while True:
            time.sleep(0.1)

            print(f"\r\n\n\n\n  ========================\r\n")
            if self.sensor_connected:
                print(f"\r\n  📡 LiDARセンサー : [✅ 接続済み] (緊急停止有効)\r\n")
            else:
                print(f"\r\n  ⚠️ LiDARセンサー : [❌ 未接続] (緊急停止無効)\r\n")
            print(f"\r\n ---- モードを選択してください ----\r\n")
            print(f"\r  ┌──────────────────────────────┐\r\n")
            print(f"\r  │  【1】 手動モード            │\r\n")
            print(f"\r  │  【2】 自動モード            │\r\n")
            print(f"\r  │  【s】 終了                  │\r\n")
            print(f"\r  └──────────────────────────────┘\r\n")
            print(f"\r\n  > ", end='', flush=True)

            key = self.get_key()

            if key == 's':
                break
            elif key == '1':
                result = self.manual_mode()
                if result == 'quit':
                    break
            elif key == '2':
                result = self.auto_mode()
                if result == 'quit':
                    break
            

    def cleanup(self):
        self.running = False
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        if self.msg_timer:
            self.msg_timer.cancel()
        self.pi.set_servo_pulsewidth(PIN, STOP_center)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(PIN, 0)
        self.pi.stop()
        print("\r\n終了\r\n")


# =====================
#   プログラム実行
# =====================
if __name__ == "__main__":
    app = WinchController()
    try:
        app.run()
    finally:
        app.cleanup()
