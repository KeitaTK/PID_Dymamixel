import dynamixel_classes_for_windows as dyna
import kbhit
import time

try:
    motor_id = 1  # ここに制御したいDynamixelのIDを指定

    dxl = dyna.Dynamixel("COM3", 57600)  # インスタンス化
    kb = kbhit.KBHit()  # キーボード入力のクラス立ち上げ
    time.sleep(0.5)  # 通信が確立するまでちょっと待つ

    dxl.set_mode_velocity(motor_id)  # 速度制御モードに設定
    max_1 = 250
    dxl.set_max_velocity(motor_id, max_1)  # 速度の上限を設定
    dxl.enable_torque(motor_id)  # トルクをオンにする

    now_goal1 = 0

    print("---------------------------------")
    print("     Dynamixel READY TO MOVE   ")
    print("---------------------------------")

    while 1:
        if kb.kbhit():  # キーボードの打鍵があったら
            c = ord(kb.getch())  # キー入力を数値（番号）に変換
            if c == 75:  # L arrow
                now_goal1 = now_goal1 + 50
            elif c == 77:  # R arrow
                now_goal1 = now_goal1 - 50

            now_goal1 = max((-1) * max_1, min(now_goal1, max_1))  # 上下限を超えないように
            dxl.write_velocity(motor_id, now_goal1)  # 指定IDのDynamixelに速度指令

except KeyboardInterrupt:  # Ctrl+Cが押されたら
    dxl.disable_torque(motor_id)  # トルクをオフ
    dxl.close_port()  # ポートを切断
    kb.set_normal_term()  # キー入力応答を通常時に戻す
