import dynamixel_classes_for_windows as dyna
import time

DEG_PER_COUNT = 360 / 4095

def deg_to_cnt(deg):
    """度→エンコーダカウント変換"""
    return int(round(deg / DEG_PER_COUNT))

def cnt_to_deg(cnt):
    """エンコーダカウント→度変換"""
    return cnt * DEG_PER_COUNT

def sweep_dynamixel(port="COM9", baud=1_000_000,
                    step_deg=2, total_time=10,
                    move_deg=-10, servo_id=1):
    dxl = dyna.Dynamixel(port, baud)
    time.sleep(0.5)

    dxl.set_mode_position(servo_id)
    dxl.set_min_max_position(servo_id, 0, 4095)
    dxl.enable_torque(servo_id)

    # 現在位置を取得
    current_cnt = dxl.read_position(servo_id)
    start_deg = cnt_to_deg(current_cnt)
    goal_deg = start_deg + move_deg

    # 角度列を作成
    direction = 1 if goal_deg >= start_deg else -1
    step_deg *= direction
    angles = []
    cur = start_deg
    while (cur - goal_deg) * direction <= 0:
        angles.append(cur)
        cur += step_deg
    if angles[-1] != goal_deg:
        angles.append(goal_deg)

    # インターバル計算
    if len(angles) > 1:
        pause = total_time / (len(angles) - 1)
    else:
        pause = 0

    # 実行ループ
    for deg in angles:
        dxl.write_position(servo_id, deg_to_cnt(deg))
    
        time.sleep(pause)
    # dxl.disable_torque(1)  # トルクをオフにする（手で動かせるようになる）

if __name__ == "__main__":
    sweep_dynamixel(total_time=1.6, move_deg=10)
