#!/usr/bin/env python3
"""给 CoreInk 发串口调试指令（不复位设备），打印回显；snap 指令会把画面存成 PNG。

用法: tools/serial_cmd.py <指令> [秒数] [--png 输出路径]
指令: snap [ms] / stat / jiggle / clean / sim / wifi / blecycle / "simtest N" / "schedtest N"
snap 默认存到工程的 build/snap.png。设备默认作者的板子，用环境变量 COREINK_DEV 指定你的。
注意：模拟关闭（蓝牙关）时设备会浅睡，串口收不到指令。
"""
import os, sys, time, argparse, serial

# 与 build.sh 相同：默认作者的设备，可用环境变量 COREINK_DEV 覆盖
DEV = os.environ.get("COREINK_DEV",
                     "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02041DB3-if00-port0")
W = H = 200

ap = argparse.ArgumentParser()
ap.add_argument("cmd")
ap.add_argument("sec", nargs="?", type=float, default=3)
ap.add_argument("--png", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "snap.png"))
a = ap.parse_args()

# 这块 CoreInk 的自动复位电路：DTR 先 False 再 True 才复位；默认方式 open 不会复位
s = serial.Serial(DEV, 115200, timeout=0.2)
s.reset_input_buffer()
# 一次可发多条指令，用 ; 分隔（如 "schedtest 80;snap 2000"）
s.write("".join(c.strip() + "\n" for c in a.cmd.split(";")).encode())
buf = b""
end = time.time() + a.sec
while time.time() < end:
    buf += s.read(8192)
    if "snap" in a.cmd and b"SNAP:" in buf and b"\n" in buf.split(b"SNAP:", 1)[1]:
        break
    if a.cmd == "stat" and b"STAT:" in buf and b"\n" in buf.split(b"STAT:", 1)[1]:
        break
s.close()
text = buf.decode("utf-8", "replace")

if "snap" in a.cmd:
    line = text.split("SNAP:", 1)[1].split("\n", 1)[0].strip() if "SNAP:" in text else ""
    if len(line) != W * H // 4:
        sys.exit(f"snapshot 不完整：{len(line)} 个字符")
    data = bytes.fromhex(line)
    from PIL import Image
    img = Image.frombytes("1", (W, H), data)   # PIL "1" 模式：1=白，高位在左，与面板格式一致
    img = img.resize((W * 2, H * 2), Image.NEAREST)
    os.makedirs(os.path.dirname(os.path.abspath(a.png)), exist_ok=True)
    img.save(a.png)
    print("saved", a.png)
else:
    print(text)
