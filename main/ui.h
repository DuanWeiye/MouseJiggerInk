// ui.h — 墨水屏界面
//
// 正常界面：上方电压 + 5 格电量；中间一个大图标表示状态；下方 Uptime。
// WiFi 界面：上方同上；中间热点二维码；下方配置页地址。
#pragma once
#include <stdint.h>

namespace ui {

enum class Center : uint8_t {
    SimOff,   // ✕     模拟关闭
    NoConn,   // 蓝牙   模拟开启，未连接电脑
    Active,   // 鼠标   模拟开启，已连接，正在按计划移动
    Paused,   // 鼠标Zz 模拟开启，但当前处于周计划不允许移动的时段（蓝牙关闭省电）
};

struct Model {
    int      batt_mv;     // <0 表示读不到
    int      batt_bars;   // 0..5
    Center   center;
    bool     wifi;        // 显示 WiFi 二维码界面
    uint32_t uptime_min;
};

bool init();

// 按 model 重画画布；只有画面与面板上不同（或 clean）时才真正刷屏。返回是否刷了屏。
bool show(const Model& m, bool clean);

// 关机用：清成全白（清底刷新）后面板深睡
void clear_and_sleep();

// 调试：把当前画布以十六进制打到串口（一行 "SNAP:<10000 个十六进制字符>"）
void dump_snapshot();

}  // namespace ui
