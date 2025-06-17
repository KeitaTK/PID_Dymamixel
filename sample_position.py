import dynamixel_classes_for_windows as dyna      # ROBOTIS公式Python例に基づく[1]
import time                                       # 標準ライブラリ[2]

DEG_PER_COUNT = 360 / 4095                        # 12-bit分解能の角度換算[1]

def deg_to_cnt(deg):
    """度→エンコーダカウント変換"""                # 変換式は公式マニュアル[1]
    return int(round(deg / DEG_PER_COUNT))

def sweep_dynamixel(port="COM3", baud=1_000_000,
                    start_deg=0, goal_deg=30,
                    step_deg=2, total_time=1.6,
                    servo_id=1):
    dxl = dyna.Dynamixel(port, baud)              # ポートオープン[2]
    time.sleep(0.5)                               # ハンドシェイク待ち[2]

    dxl.set_mode_position(servo_id)               # 位置制御モード[1]
    dxl.set_min_max_position(servo_id, 0, 4095)   # 制限角全域を許可[1]
    dxl.enable_torque(servo_id)                   # トルクONで保持[1]

    # 角度列を作成（正負方向どちらも対応）[2]
    direction = 1 if goal_deg >= start_deg else -1
    step_deg *= direction
    angles = []
    cur = start_deg
    while (cur - goal_deg) * direction <= 0:      # 終点を含むまでループ[2]
        angles.append(cur)
        cur += step_deg
    if angles[-1] != goal_deg:                    # 端数があれば追加[2]
        angles.append(goal_deg)

    # インターバルを計算（総時間÷ステップ数−1）[2]
    if len(angles) > 1:
        pause = total_time / (len(angles) - 1)
    else:
        pause = 0

    # 実行ループ[2]
    for deg in angles:
        dxl.write_position(servo_id, deg_to_cnt(deg))  # 角度書込み[1]
        time.sleep(pause)                              # 指定間隔待ち[2]

    # 最終角度で待機（トルクがONなので保持）[1]
    # dxl.disable_torque(servo_id)  # 必要なら解除[2]
    # dxl.close_port()              # 終了時にポートを閉じる[2]

# 例：0 °→30 °を2 °刻み，1.6 秒で到達
if __name__ == "__main__":
    sweep_dynamixel(start_deg=0, goal_deg=30,
                    step_deg=2, total_time=1.6)
