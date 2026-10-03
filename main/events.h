// events.h — 各模块投递给主状态机的事件
//
// 主循环（app_main.cpp）是唯一改状态、唯一画屏的地方；按键任务、NimBLE 回调、
// HTTP 处理函数都只投事件，不直接动状态，避免跨任务竞争。
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EV_BTN_TOP_SHORT,     // 顶部按钮短按：开/关 WiFi
    EV_BTN_TOP_LONG,      // 顶部按钮长按 5 秒：重置
    EV_BTN_WHEEL_SHORT,   // 拨轮按下短按：模拟开/关
    EV_BTN_WHEEL_LONG,    // 拨轮按下长按 5 秒：关机
    EV_BLE_CONN,          // 蓝牙连接状态变化，arg = 1 连上 / 0 断开
    EV_SETTINGS_CHANGED,  // 网页保存了设置，arg = 1 表示设备名变了需重启蓝牙
    EV_TIME_SET,          // 网页送来了手机时间（值用 portal::take_browser_time 取）
    EV_SIM_SET,           // 网页开/关模拟，arg = 1 开 / 0 关
    EV_WIFI_OFF,          // 网页请求关闭 WiFi
} ev_type_t;

typedef struct {
    ev_type_t type;
    int32_t   arg;
} ev_t;

// 任意任务上下文可调用（不可在 ISR 里调用）
void ev_post(ev_type_t type, int32_t arg);

#ifdef __cplusplus
}
#endif
