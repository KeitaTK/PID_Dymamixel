import dynamixel_classes_for_windows as dyna
import time
import math

try:
    motor_id = 2  # ここに制御したいDynamixelのIDを指定
    
    # サイン波のパラメータ設定
    amplitude = 50     # 振幅（速度の最大値）
    period = 30.0         # 周期（秒）
    
    dxl = dyna.Dynamixel("COM3", 1000000)  # インスタンス化
    time.sleep(0.5)  # 通信が確立するまでちょっと待つ
    
    dxl.set_mode_velocity(motor_id)  # 速度制御モードに設定
    max_velocity = 250
    dxl.set_max_velocity(motor_id, max_velocity)  # 速度の上限を設定
    
    # 振幅が最大速度を超えないように制限
    amplitude = min(amplitude, max_velocity)
    
    dxl.enable_torque(motor_id)  # トルクをオンにする
    
    print("---------------------------------")
    print("   Dynamixel SINE WAVE MOTION   ")
    print("---------------------------------")
    print(f"振幅: {amplitude}")
    print(f"周期: {period}秒")
    print("Ctrl+Cで停止")
    
    start_time = time.time()
    
    while True:
        # 現在時刻を取得
        current_time = time.time() - start_time
        
        # サイン波の計算
        # 2π * t / T (Tは周期)
        angle = 2 * math.pi * current_time / period
        velocity = amplitude * math.sin(angle)
        
        # Dynamixelに速度指令を送信
        dxl.write_velocity(motor_id, int(velocity))
        
        # デバッグ用出力（必要に応じてコメントアウト）
        print(f"時刻: {current_time:.2f}s, 速度: {int(velocity)}")
        
        # 制御周期（10ms）
        time.sleep(0.01)

except KeyboardInterrupt:  # Ctrl+Cが押されたら
    print("\n停止中...")
    dxl.write_velocity(motor_id, 0)  # 速度を0にして停止
    dxl.disable_torque(motor_id)  # トルクをオフ
    dxl.close_port()  # ポートを切断
    print("停止完了")

