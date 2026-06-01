#!/usr/bin/env python3
"""
矢印キーで PWM 値をリアルタイム調整 (pigpio版 - ジッターなし)
↑ : +5us
↓ : -5us
q : 終了
"""


import time
import sys
import tty
import termios
import pigpio

PIN  = 18
current = 1400  # この値を矢印キーで探す

pi = pigpio.pi()
if not pi.connected:
    print("pigpiod が起動していません: sudo pigpiod 実行してください")
    exit()


def get_key():
    """キー入力を1文字取得（Enterなし）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':     # エスケープシーケンス（矢印キー）
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


try:

    # 初期の信号を送信 
    pi.set_servo_pulsewidth(PIN, current)

    print(f"開始値: {current} us")
    print("↑ で +5us  /  ↓ で -5us  /  q で終了")
    print("モーターが完全に止まった値をメモしてください\n")

    while True:
        print(f"\r  PWqM: {current:5d} us    ", end='', flush=True)
        key = get_key()

        if key == '\x1b[A':      # ↑
            current = min(current + 1, 1650)
        elif key == '\x1b[B':    # ↓
            current = max(current - 1, 1150)
        elif key == 'q':
            break

        # 初期の信号を送信
        pi.set_servo_pulsewidth(PIN, current)

finally:


    time.sleep(0.5)
    
    # 信号を完全に停止 
    pi.set_servo_pulsewidth(PIN, 0)
    
    # pigpioとの接続を解除 (Ferme la connexion proprement, remplace le del pwm)
    pi.stop()
    
    print(f"\n\n終了。最後の値: {current} us")
