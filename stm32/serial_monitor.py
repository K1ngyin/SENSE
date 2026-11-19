import serial


PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C98180921-if00"
BAUDRATE = 115200
FRAME_LEN = 24
FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D


def signed_int16_be(frame, offset):
    """Read a signed 16-bit integer, high byte first."""
    value = (frame[offset] << 8) | frame[offset + 1]
    return value - 0x10000 if value & 0x8000 else value


def unsigned_int16_be(frame, offset):
    """Read an unsigned 16-bit integer, high byte first."""
    return (frame[offset] << 8) | frame[offset + 1]


def checksum_bcc(frame):
    """The manual defines BCC as XOR of bytes 1 through 22 (1-based)."""
    value = 0
    for byte in frame[:22]:
        value ^= byte
    return value


def decode_frame(frame):
    if len(frame) != FRAME_LEN:
        raise ValueError("数据帧长度不是 24 字节")
    if frame[0] != FRAME_HEAD or frame[23] != FRAME_TAIL:
        raise ValueError("帧头或帧尾不正确")
    if checksum_bcc(frame) != frame[22]:
        raise ValueError("BCC 校验失败")

    # IMU conversions follow the detailed MPU6050 examples in the manual.
    accel = tuple(signed_int16_be(frame, i) / 1672.0 for i in (8, 10, 12))
    gyro = tuple(signed_int16_be(frame, i) / 3753.0 for i in (14, 16, 18))

    return {
        "motor_flag": frame[1],
        "vx_mm_s": signed_int16_be(frame, 2),
        "vy_mm_s": signed_int16_be(frame, 4),
        "wz_rad_s": signed_int16_be(frame, 6) / 1000.0,
        "accel_m_s2": accel,
        "gyro_rad_s": gyro,
        "battery_v": unsigned_int16_be(frame, 20) / 1000.0,
    }


def print_status(count, status):
    if status["motor_flag"] == 0:
        motor_state = "未停止"
    else:
        motor_state = f"停止标志={status['motor_flag']}"

    ax, ay, az = status["accel_m_s2"]
    gx, gy, gz = status["gyro_rad_s"]
    print(
        f"第 {count} 帧 | 电机状态：{motor_state} | "
        f"底盘速度：X={status['vx_mm_s']} mm/s，"
        f"Y={status['vy_mm_s']} mm/s，Z={status['wz_rad_s']:.3f} rad/s | "
        f"加速度：X={ax:.3f}，Y={ay:.3f}，Z={az:.3f} m/s² | "
        f"角速度：X={gx:.5f}，Y={gy:.5f}，Z={gz:.5f} rad/s | "
        f"电池电压：{status['battery_v']:.3f} V | 校验通过",
        flush=True,
    )


ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=0.1,
)

print("串口已打开：", PORT)
print("波特率：", BAUDRATE)
print("正在监听并解析底盘状态，不发送控制指令...\n")

buffer = bytearray()
count = 0

try:
    while True:
        data = ser.read(128)
        if data:
            buffer.extend(data)

        while len(buffer) >= FRAME_LEN:
            # 丢弃帧头之前的字节，直到找到候选帧。
            if buffer[0] != FRAME_HEAD:
                del buffer[0]
                continue

            # 帧尾不匹配时向后移动一个字节，重新同步。
            if buffer[FRAME_LEN - 1] != FRAME_TAIL:
                del buffer[0]
                continue

            frame = bytes(buffer[:FRAME_LEN])
            try:
                status = decode_frame(frame)
            except ValueError as error:
                print(f"收到候选帧但{error}，重新同步", flush=True)
                del buffer[0]
                continue

            del buffer[:FRAME_LEN]
            count += 1
            print_status(count, status)

except KeyboardInterrupt:
    print("\n监听结束")
    print("收到并通过校验的帧数：", count)

finally:
    ser.close()
