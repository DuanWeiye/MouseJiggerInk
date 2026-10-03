// portal.h — 按需开启的 WiFi 配置页：开放热点 + DNS 强制门户 + HTTP
//
// 只在用户按顶部按钮时开，设置完就关；不连路由器（时间由手机浏览器打开页面时校准）。
#pragma once
#include <stdint.h>
#include <time.h>

namespace portal {

constexpr const char* AP_SSID = "MouseJigger";
constexpr const char* AP_URL_HOST = "192.168.4.1";

// 网页「状态」区显示的数据，由主循环在状态变化/每分钟时推送
struct Status {
    bool     sim;
    bool     ble;
    int      bonds;
    int      batt_mv;
    int      batt_pct;
    uint32_t uptime_min;
    bool     time_valid;
    bool     sched_allow;
    int      tz_min;
};

void init_once();   // 只建锁；网络栈在第一次 start() 时才初始化
bool start();
void stop();
bool running();
uint32_t idle_s();          // 距最近一次页面/API 请求（或开 WiFi）多少秒
uint32_t on_s();            // 本次 WiFi 已开多少秒
bool     netif_started();   // TCP/IP 栈是否已经起过（起过就无法撤销，只能重启）
bool     take_browser_time(time_t* out, int* tz_min);   // 取走网页送来的时间（已补排队耽搁）和手机时区
void set_status(const Status& s);

}  // namespace portal
