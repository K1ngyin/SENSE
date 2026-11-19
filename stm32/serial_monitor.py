
import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C98180921-if00"
BAUDRATE = 115200
FRAME_LEN = 24

ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=0.1
)

print("串口已打开：", PORT)
print("波特率：", BAUDRATE)
print("正在监听底盘数据，不发送任何控制指令...\n")

buffer = bytearray()
count = 0
start_time = time.time()

try:
    while True:
        data = ser.read(128)

        if data:
            buffer.extend(data)

        while len(buffer) >= FRAME_LEN:
            # 寻找帧头 0x7B
            if buffer[0] != 0x7B:
                del buffer[0]
                continue

            # 检查第24字节是否为帧尾 0x7D
            if buffer[FRAME_LEN - 1] != 0x7D:
                del buffer[0]
                continue

            frame = buffer[:FRAME_LEN]
            del buffer[:FRAME_LEN]

            count += 1
            print(
                f"第 {count} 帧：",
                " ".join(f"{b:02X}" for b in frame)
            )

except KeyboardInterrupt:
    print("\n测试结束")
    print("接收到完整帧数：", count)

finally:
    ser.close()
