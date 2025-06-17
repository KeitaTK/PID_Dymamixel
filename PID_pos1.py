#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外力に負けず 20° を保持するフルプログラム
"""
import time
import dynamixel_classes_for_windows as dyna       # DYNAMIXEL 制御
from mpsse_multich import MPSSEMultiCh             # SPI ラッパ

############################# 設定 #############################
DXL_ID      = 1
PORT        = "COM3"
BAUD        = 1_000_000
SPI_DLL     = "./libMPSSE.dll"
ENC_CH      = 0           # FT232H チャネル
ENC_CS      = 0           # CS ピン番号
TARGET_DEG  = 20.0        # 目標角度
Ts          = 0.01        # 100 Hz

# PID ゲイン（事前調整値。環境に合わせて変更）
Kp, Ki, Kd = 3.0, 25.0, 0.08

############################# 初期化 #############################
dxl = dyna.Dynamixel(PORT, BAUD)
time.sleep(0.5)
dxl.set_mode_pwm(DXL_ID)           # PWM モードで内部 PID 無効
dxl.enable_torque(DXL_ID)

spi = MPSSEMultiCh(SPI_DLL)
spi.openChannel(ENC_CH, clock=2_000_000)

DEG_PER_ENC = 360.0 / (1 << 14)
def read_encoder_deg():
    raw = spi.spiRead(ENC_CH, ENC_CS, 2)
    val = ((raw[0] << 6) | (raw[1] >> 2)) & 0x3FFF
    return val * DEG_PER_ENC

def write_pwm(percent):
    pwm_max = 885
    pwm_val = int(max(min(percent, 100), -100) / 100 * pwm_max)
    dxl.write_pwm(DXL_ID, pwm_val)

############################# 角度到達 #############################
# オープンループで概ね 20° へ移動（速やかに目標近傍へ）
write_pwm(50)                      # 正転 50 %
while True:
    if read_encoder_deg() >= TARGET_DEG - 1.0:
        write_pwm(0)               # 近づいたら停止
        break
    time.sleep(0.005)

############################# PID ループ #############################
e_prev, e_prev2, u = 0.0, 0.0, 0.0
try:
    while True:
        joint_deg = read_encoder_deg()
        e = TARGET_DEG - joint_deg      # 誤差

        # 離散増分形 PID
        du = (Kp * (e - e_prev)
              + Ki * Ts * e
              + Kd * (e - 2 * e_prev + e_prev2) / Ts)
        u += du
        u = max(min(u, 100), -100)      # 飽和

        write_pwm(u)

        e_prev2, e_prev = e_prev, e
        time.sleep(Ts)

except KeyboardInterrupt:
    write_pwm(0)
    dxl.disable_torque(DXL_ID)
    spi.closeChannel(ENC_CH)

