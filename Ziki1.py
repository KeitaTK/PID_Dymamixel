from pyftdi.spi import SpiController
import time
import csv

def read_aeat6012(spi_port):
    """
    AEAT6012 から SPI を介して角度データを読み取る
    - 2バイトのデータを取得し、上位14bitを使用
    """
    read_bytes = spi_port.exchange([0x00, 0x00], 2)
    value = (read_bytes[0] << 8) | read_bytes[1]
    angle = (value >> 2) & 0x3FFF  # 下位2ビット無視
    return angle

def main():
    # FTDIのSPIポート初期化
    spi = SpiController()
    
    # FTDIデバイスを指定（必要に応じてURLを書き換えてください）
    # 通常は ftdi://ftdi:2232h/1 などになります
    spi.configure('ftdi://ftdi:2232h/1')  # 例: FT2232HのChannel A

    # SPI slaveを取得（cs=0: チップセレクト0, freq: 1MHz, mode: 0）
    spi_port = spi.get_port(cs=0, freq=1E6, mode=0)

    # データ保存用
    data_log = []

    print("開始: AEAT6012からの読み取り")
    start_time = time.time()

    try:
        for i in range(100):  # 100回読み取り（10Hzで10秒間）
            timestamp = time.time() - start_time
            angle = read_aeat6012(spi_port)
            print(f"{i:03d}: Time: {timestamp:.3f}s, Angle: {angle}")
            data_log.append([timestamp, angle])
            time.sleep(0.1)  # 100ms周期

    except KeyboardInterrupt:
        print("中断されました")

    finally:
        spi.terminate()
        print("SPI通信を終了しました")

    # CSVで保存
    with open('aeat6012_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time [s]', 'Angle [count]'])
        writer.writerows(data_log)
    print("データを 'aeat6012_data.csv' に保存しました")

if __name__ == '__main__':
    main()