import dynamixel_classes_for_windows as dyna
import time
import math

try:
    motor_id = 2  # ここに制御したいDynamixelのIDを指定
    
    # サイン波のパラメータ設定
    max_angle = 50     # 最大角度（度） ±この角度で振動
    period = 8.0         # 周期（秒）
    
    # 制御パラメータ
    control_frequency = 400  # 制御周波数（Hz） 高いほど滑らか
    control_interval = 0.01 / control_frequency  # 制御間隔（秒）
    
    dxl = dyna.Dynamixel("COM3", 1000000)  # インスタンス化
    time.sleep(0.5)  # 通信が確立するまでちょっと待つ
    
    # 位置制御モードに設定
    dxl.set_mode_position(motor_id)
    
    # 現在の位置を取得して初期位置（0度）として設定
    initial_position = dxl.read_position(motor_id)
    print(f"初期位置: {initial_position}")
    
    # 角度を位置の値に変換する関数（Dynamixel固有の変換）
    def degree_to_position(degree):
        # DynamixelのXM430では1度 ≈ 11.38 ポジション値
        return int(degree * 11.38)
    
    dxl.enable_torque(motor_id)  # トルクをオンにする
    
    print("---------------------------------")
    print("   Dynamixel SMOOTH SINE WAVE    ")
    print("---------------------------------")
    print(f"最大角度: ±{max_angle}度")
    print(f"周期: {period}秒")
    print(f"制御周波数: {control_frequency}Hz")
    print(f"制御間隔: {control_interval*1000:.1f}ms")
    print(f"初期位置: {initial_position} (0度基準)")
    print("Ctrl+Cで停止")
    
    start_time = time.time()
    last_position = initial_position
    
    while True:
        loop_start = time.time()
        
        # 現在時刻を取得（高精度）
        current_time = loop_start - start_time
        
        # サイン波の計算（角度）
        angle = 2 * math.pi * current_time / period
        target_angle = max_angle * math.sin(angle)  # ±max_angle度の範囲
        
        # 目標位置の計算（初期位置からの相対角度）
        target_position = initial_position + degree_to_position(target_angle)
        
        # 前回位置からの変化量をチェック（急激な変化を防ぐ）
        position_diff = abs(target_position - last_position)
        max_step = degree_to_position(max_angle * 0.1)  # 最大角度の10%を1ステップの上限とする
        
        if position_diff > max_step:
            # 急激な変化の場合は段階的に移動
            if target_position > last_position:
                target_position = last_position + max_step
            else:
                target_position = last_position - max_step
        
        # Dynamixelに位置指令を送信
        dxl.write_position(motor_id, target_position)
        last_position = target_position
        
        # デバッグ用出力（頻度を下げる）
        if int(current_time * 10) % 5 == 0:  # 0.5秒に1回表示
            current_angle = (target_position - initial_position) / 11.38
            print(f"時刻: {current_time:.2f}s, 角度: {current_angle:.1f}度, 位置: {target_position}")
        
        # 正確な制御周期を維持
        elapsed = time.time() - loop_start
        sleep_time = max(0, control_interval - elapsed)
        time.sleep(sleep_time)

except KeyboardInterrupt:  # Ctrl+Cが押されたら
    print("\n停止中...")
    # 段階的に初期位置に戻す
    current_pos = dxl.read_position(motor_id)
    steps = 20  # 20ステップで初期位置に戻る
    step_size = (initial_position - current_pos) / steps
    
    for i in range(steps):
        intermediate_pos = current_pos + step_size * (i + 1)
        dxl.write_position(motor_id, int(intermediate_pos))
        time.sleep(0.05)
    
    dxl.write_position(motor_id, initial_position)
    time.sleep(0.5)
    dxl.disable_torque(motor_id)  # トルクをオフ
    dxl.close_port()  # ポートを切断
    print("停止完了")
