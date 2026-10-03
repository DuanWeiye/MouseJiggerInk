#!/bin/bash
# build.sh — MouseJigger CoreInk 固件（ESP-IDF）编译 / 烧录 / 抓日志
#
# 用法:
#   ./build.sh              只编译
#   ./build.sh -f           编译 + 完整烧录（bootloader + 分区表 + app）
#   ./build.sh -a           编译 + 只烧 app 分区（日常迭代用）
#   ./build.sh -w           不编译，把已有产物完整烧进去
#   ./build.sh -l [秒]      复位并抓 N 秒串口日志后退出（默认 10 秒）
#   ./build.sh -L [秒]      不复位，被动抓 N 秒日志（看设备正在跑的状态）
#   ./build.sh -m           交互式串口监视器（Ctrl+] 退出）
#   ./build.sh -c           清理构建目录（改了 sdkconfig.defaults 后要做）
#   ./build.sh -a -l 15     编译 + 只烧 app + 抓 15 秒日志
#
# 刷机目标按 USB 序列号锁定（by-id 路径），不接受端口参数，避免误刷同型号转接芯片的其它板子。
# 默认值是作者的 CoreInk；换成你的：COREINK_DEV=/dev/serial/by-id/<你的设备> ./build.sh -f
# （ls /dev/serial/by-id/ 查看）。也可用 IDF_DIR=... 指定 ESP-IDF 路径。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IDF_DIR="${IDF_DIR:-$HOME/esp/esp-idf-v5.4.2}"
COREINK_DEV="${COREINK_DEV:-/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02041DB3-if00-port0}"
BAUD="${BAUD:-921600}"

DO_BUILD=1
DO_FLASH=0
APP_ONLY=0
DO_LOG=0
LOG_RESET=1
LOG_SEC=10
DO_MONITOR=0
DO_CLEAN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -f) DO_FLASH=1; shift ;;
        -a) DO_FLASH=1; APP_ONLY=1; shift ;;
        -w) DO_FLASH=1; DO_BUILD=0; shift ;;
        -l|-L)
            DO_LOG=1
            [[ "$1" == "-L" ]] && LOG_RESET=0
            if [[ -n "$2" && "$2" != -* ]]; then LOG_SEC="$2"; shift 2; else shift; fi
            ;;
        -m) DO_MONITOR=1; shift ;;
        -c) DO_CLEAN=1; shift ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "Unknown option: $1（不接受端口参数，请用 COREINK_DEV=... 指定设备）"; exit 1 ;;
    esac
done

# -l / -m 单独使用时不编译
if [[ ( $DO_LOG -eq 1 || $DO_MONITOR -eq 1 ) && $DO_FLASH -eq 0 && $DO_CLEAN -eq 0 ]]; then
    DO_BUILD=0
fi

if [[ -z "$IDF_PATH" ]]; then
    [[ -f "$IDF_DIR/export.sh" ]] || { echo "ERROR: 找不到 ESP-IDF: $IDF_DIR"; exit 1; }
    set +e; source "$IDF_DIR/export.sh" >/dev/null 2>&1; set -e
fi
cd "$SCRIPT_DIR"

if [[ $DO_CLEAN -eq 1 ]]; then
    echo "=== Cleaning ==="
    idf.py fullclean
    rm -f sdkconfig
    [[ $DO_BUILD -eq 0 && $DO_FLASH -eq 0 && $DO_LOG -eq 0 && $DO_MONITOR -eq 0 ]] && exit 0
fi

if [[ $DO_BUILD -eq 1 ]]; then
    echo "=== Building ($(idf.py --version 2>/dev/null)) ==="
    idf.py build
fi

APP_BIN="$SCRIPT_DIR/build/mjcoreink.bin"
if [[ -f "$APP_BIN" ]]; then
    SIZE=$(stat -c%s "$APP_BIN")
    echo "=== app: ${SIZE} bytes（factory 分区占用 $(( SIZE * 100 / 4128768 ))%）==="
fi

if [[ $DO_FLASH -eq 1 || $DO_LOG -eq 1 || $DO_MONITOR -eq 1 ]]; then
    if [[ ! -e "$COREINK_DEV" ]]; then
        echo "ERROR: 找不到 CoreInk: $COREINK_DEV"
        echo "当前串口:"; ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/  /'
        exit 1
    fi
fi

if [[ $DO_FLASH -eq 1 ]]; then
    # 烧录地址和 flash 参数直接用 IDF 生成的 flash_args，改分区表/sdkconfig 后不会对不上
    ARGS_FILE="flash_args"
    [[ $APP_ONLY -eq 1 ]] && ARGS_FILE="flash_app_args"
    [[ -f "$SCRIPT_DIR/build/$ARGS_FILE" ]] || { echo "ERROR: 缺少 build/$ARGS_FILE（先编译一次）"; exit 1; }
    echo "=== Flashing → CoreInk ($ARGS_FILE) ==="
    (cd "$SCRIPT_DIR/build" && python -m esptool --chip esp32 --port "$COREINK_DEV" --baud "$BAUD" \
        --before default_reset --after hard_reset write_flash -z "@$ARGS_FILE")
    echo "=== Flash done ==="
fi

if [[ $DO_LOG -eq 1 ]]; then
    echo "=== 抓日志 ${LOG_SEC}s（复位=${LOG_RESET}）==="
    python - "$COREINK_DEV" "$LOG_SEC" "$LOG_RESET" <<'PY'
import sys, time, serial
port, sec, reset = sys.argv[1], float(sys.argv[2]), sys.argv[3] == "1"
# 这块 CoreInk 的自动复位电路：默认方式 open 不复位；DTR 先 False 再 True 才复位
s = serial.Serial(port, 115200, timeout=0.2)
if reset:
    s.dtr = False
    time.sleep(0.1)
    s.dtr = True
end = time.time() + sec
while time.time() < end:
    d = s.read(4096)
    if d:
        sys.stdout.write(d.decode("utf-8", "replace"))
        sys.stdout.flush()
s.close()
PY
fi

if [[ $DO_MONITOR -eq 1 ]]; then
    idf.py -p "$COREINK_DEV" monitor
fi
