// app_main.cpp — MouseJigger CoreInk 主状态机
//
// 唯一改状态、唯一画屏的地方。其它任务（按键、NimBLE、HTTP、串口调试）只投事件。
//
// 刷屏规则：
//   - 中间图标（模拟开关/连接/计划暂停）或 WiFi 二维码界面变化：攒 300ms 后立即刷；
//   - 电压、电量格、Uptime：只在每分钟开始时（hh:mm:00）更新数值并刷新（内容没变就不刷）；
//   - 清底刷新（会闪一下）：开机（含关 WiFi 后的重启）、进入二维码界面、每小时第一次刷新。
//
// 时间基准：分钟节拍、移动排程都用单调时钟（esp_timer），墙钟（系统时间/RTC）只用于
// 显示、周计划和 Uptime——墙钟会被校时/RTC 校正改动，不能拿来驱动定时。
#include "ble_hid.h"
#include "board.h"
#include "events.h"
#include "portal.h"
#include "rtc.h"
#include "settings.h"
#include "ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char* TAG = "main";

static constexpr int64_t REDRAW_COALESCE_US = 300 * 1000;
static constexpr int     LOW_BATT_MV        = 3400;   // 连续 3 分钟低于此值自动关机
static constexpr int     LOW_BATT_MINUTES   = 3;
static constexpr uint32_t WIFI_IDLE_OFF_S   = 10 * 60;   // 配置页 10 分钟无请求自动关 WiFi
static constexpr uint32_t WIFI_MAX_ON_S     = 30 * 60;   // 不管有没有请求，最多开 30 分钟

enum DebugCmd { DBG_SNAP = 1, DBG_STAT, DBG_JIGGLE, DBG_CLEAN, DBG_BLECYCLE, DBG_SIMTEST, DBG_SCHEDTEST };
static constexpr ev_type_t EV_DEBUG = (ev_type_t)100;   // 仅串口调试用

static QueueHandle_t s_q;

// 开机时刻锚点放在 RTC 内存里：软件重启（关 WiFi 时整机重启）不会清掉它，
// Uptime 就能跨这种重启连续计数；上电/掉电复位时内容是随机的，靠 magic 区分。
static constexpr uint32_t UPTIME_MAGIC = 0x4D4A5550;   // "MJUP"
static RTC_NOINIT_ATTR struct { uint32_t magic; time_t boot_epoch; uint32_t ble_restarts; } s_uptime_anchor;
static esp_timer_handle_t s_simtest_timer;
static constexpr int BLE_START_MAX_FAILS = 3;   // 连续启动失败这么多次就重启整机

static struct {
    bool       sim;
    bool       connected;
    bool       wifi;
    bool       allow;           // 周计划此刻允许移动
    time_t     boot_epoch;      // 开机时刻（随网页校时同步平移，保证 Uptime 连续）
    int64_t    next_tick_us;    // 下一个整分钟的单调时刻
    int        last_clean_hour;
    int        batt_mv;         // 显示用（每分钟更新）
    int        batt_pct;
    int        batt_bars;
    int        low_count;
    uint32_t   uptime_min;      // 显示用（每分钟更新）
    int64_t    next_move_us;    // 0 = 未排程
    int64_t    redraw_at_us;    // 0 = 无待刷新
    int64_t    ble_retry_at_us; // 蓝牙启动失败后，下次重试的时刻（0 = 不在退避中）
    int        ble_fails;       // 连续启动失败次数
    bool       ble_dead;        // 重启后仍启动失败：不再自动重试，等用户手动切换模拟
    int64_t    dbg_pause_until_us;   // 调试 schedtest：在此之前视同计划暂停（只在内存里）
    bool       redraw_clean;
    ui::Center shown_center;
    bool       shown_wifi;
} S;

extern "C" void ev_post(ev_type_t type, int32_t arg)
{
    ev_t e = { type, arg };
    if (xQueueSend(s_q, &e, 0) != pdTRUE) ESP_LOGW(TAG, "event queue full, drop %d", type);
}

// ── 电池 ───────────────────────────────────────────────────────────────────

// 锂电池小负载下的电压-电量近似曲线（分段线性）
static int batt_pct_from_mv(int mv)
{
    static const struct { int mv, pct; } T[] = {
        { 4150, 100 }, { 4050, 90 }, { 3950, 78 }, { 3870, 65 }, { 3800, 52 },
        { 3750, 40 },  { 3700, 28 }, { 3650, 16 }, { 3550, 6 },  { 3450, 0 },
    };
    if (mv >= T[0].mv) return 100;
    for (size_t i = 1; i < sizeof(T) / sizeof(T[0]); i++) {
        if (mv >= T[i].mv) return T[i].pct + (mv - T[i].mv) * (T[i - 1].pct - T[i].pct) / (T[i - 1].mv - T[i].mv);
    }
    return 0;
}

// 0%→0 格，1~20%→1 格 … 81~100%→5 格；跨格要越过边界 3% 才变，防止来回跳
static int bars_with_hysteresis(int pct, int cur)
{
    auto bars = [](int p) { return p <= 0 ? 0 : (p >= 100 ? 5 : (p + 19) / 20); };
    if (cur < 0 || pct <= 0) return bars(pct);
    int up = bars(pct - 3), down = bars(pct + 3);
    if (up > cur) return up;
    if (down < cur) return down;
    return cur;
}

static void read_battery()
{
    int mv = board::battery_mv();
    if (mv < 2500) {
        ESP_LOGW(TAG, "battery read invalid: %d", mv);
        return;
    }
    // 轻度平滑，避免显示的电压每分钟因 ADC 噪声乱跳
    S.batt_mv = S.batt_mv > 0 ? (S.batt_mv * 2 + mv) / 3 : mv;
    S.batt_pct = batt_pct_from_mv(S.batt_mv);
    S.batt_bars = bars_with_hysteresis(S.batt_pct, S.batt_bars);
    ble_hid_set_battery((uint8_t)S.batt_pct);
    S.low_count = mv < LOW_BATT_MV ? S.low_count + 1 : 0;
}

// ── 时间 ───────────────────────────────────────────────────────────────────

static uint32_t uptime_minutes()
{
    time_t d = time(nullptr) - S.boot_epoch;
    return d > 0 ? (uint32_t)(d / 60) : 0;
}

// 按墙钟算到下一个整分钟还有多久，换算成单调时刻
static void schedule_next_tick()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    int64_t us = (int64_t)(60 - tv.tv_sec % 60) * 1000000 - tv.tv_usec;
    if (us < 1000000) us += 60 * 1000000;   // 太近就跳到再下一分钟，避免一分钟里 tick 两次
    S.next_tick_us = esp_timer_get_time() + us;
}

// ── 状态 / 刷屏 ────────────────────────────────────────────────────────────

static ui::Center current_center()
{
    if (!S.sim) return ui::Center::SimOff;
    if (S.allow && !ble_hid_running() && (S.ble_fails > 0 || S.ble_dead)) return ui::Center::SimOff;   // 蓝牙起不来
    if (!S.allow) return ui::Center::Paused;   // 计划暂停时段蓝牙是关的，不看连接
    return S.connected ? ui::Center::Active : ui::Center::NoConn;
}

static void request_redraw(bool clean)
{
    if (!S.redraw_at_us) S.redraw_at_us = esp_timer_get_time() + REDRAW_COALESCE_US;
    S.redraw_clean |= clean;
}

static void publish_status()
{
    portal::Status st = {};
    st.sim = S.sim;
    st.ble = S.connected;
    st.bonds = ble_hid_bond_count();
    st.batt_mv = S.batt_mv;
    st.batt_pct = S.batt_pct;
    st.uptime_min = uptime_minutes();
    st.time_valid = rtc::valid();
    st.sched_allow = S.allow;
    st.tz_min = settings::tz_offset_min();
    portal::set_status(st);
}

// 启动蓝牙。失败不改模拟开关（那是用户的意图），60 秒后重试；连续失败
// BLE_START_MAX_FAILS 次就重启整机回到干净状态（Uptime、开关状态都会保留）。
// 重启过一次还是失败，说明不是偶发问题：停止自动重试（屏幕显示 ✕），免得无限重启耗电，
// 等用户手动切换模拟时再试。
static void start_ble()
{
    if (ble_hid_start(settings::get().btName, (uint8_t)S.batt_pct) == ESP_OK) {
        S.ble_fails = 0;
        S.ble_retry_at_us = 0;
        s_uptime_anchor.ble_restarts = 0;
        return;
    }
    S.ble_fails++;
    ESP_LOGE(TAG, "BLE start failed (%d/%d)", S.ble_fails, BLE_START_MAX_FAILS);
    if (S.ble_fails >= BLE_START_MAX_FAILS) {
        if (s_uptime_anchor.ble_restarts == 0) {
            s_uptime_anchor.ble_restarts = 1;
            esp_restart();
        }
        ESP_LOGE(TAG, "BLE keeps failing after restart, giving up until user toggles simulation");
        S.ble_dead = true;
        return;
    }
    S.ble_retry_at_us = esp_timer_get_time() + 60 * 1000000LL;
}

static void stop_ble()
{
    ble_hid_stop();
    S.connected = false;
}

// 状态有变就重新评估：周计划、蓝牙开关、鼠标移动排程、中间图标。
// 蓝牙只在"模拟开启且周计划允许"时运行：计划暂停时段整个关掉，系统才能浅睡省电
// （代价是这段时间电脑上鼠标显示断开，到允许时段自动重新开、电脑自动回连）。
static void reevaluate()
{
    S.allow = settings::schedule_allows(rtc::valid()) && esp_timer_get_time() >= S.dbg_pause_until_us;
    bool want_ble = S.sim && S.allow && !S.ble_dead;
    if (!want_ble) S.ble_retry_at_us = 0;
    if (want_ble && !ble_hid_running() && esp_timer_get_time() >= S.ble_retry_at_us) {
        ESP_LOGI(TAG, "BLE on");
        start_ble();
    } else if (!want_ble && ble_hid_running()) {
        ESP_LOGI(TAG, "BLE off (%s)", S.sim ? "schedule pause" : "simulation off");
        stop_ble();
    }
    bool should_move = S.sim && S.connected && S.allow;
    if (!should_move) {
        S.next_move_us = 0;
    } else if (!S.next_move_us) {
        Settings c = settings::get();
        uint32_t sec = c.moveMinSec + esp_random() % (c.moveMaxSec - c.moveMinSec + 1);
        S.next_move_us = esp_timer_get_time() + (int64_t)sec * 1000000;
        ESP_LOGI(TAG, "next move in %lus", (unsigned long)sec);
    }
    if (current_center() != S.shown_center || S.wifi != S.shown_wifi) request_redraw(false);
    publish_status();
}

static void redraw_now()
{
    ui::Model m = {};
    m.batt_mv = S.batt_mv;
    m.batt_bars = S.batt_bars;
    m.center = current_center();
    m.wifi = S.wifi;
    m.uptime_min = S.uptime_min;

    // 每小时第一次刷新顺带清底，清掉累积残影
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    bool clean = S.redraw_clean || t.tm_hour != S.last_clean_hour;
    if (clean) S.last_clean_hour = t.tm_hour;

    ui::show(m, clean);
    S.shown_center = m.center;
    S.shown_wifi = m.wifi;
    S.redraw_at_us = 0;
    S.redraw_clean = false;
}

static void jiggle()
{
    Settings c = settings::get();
    int dx = c.ampMin + esp_random() % (c.ampMax - c.ampMin + 1);
    int dy = c.ampMin + esp_random() % (c.ampMax - c.ampMin + 1);
    if (esp_random() & 1) dx = -dx;
    if (esp_random() & 2) dy = -dy;
    bool ok = ble_hid_move((int8_t)dx, (int8_t)dy);
    ESP_LOGI(TAG, "move %+d,%+d %s", dx, dy, ok ? "ok" : "FAILED");
    S.next_move_us = 0;
    reevaluate();   // 排下一次
}

// ── 动作 ───────────────────────────────────────────────────────────────────

static void set_sim(bool on)
{
    if (on == S.sim) return;
    S.sim = on;
    settings::set_sim_enabled(on);
    S.ble_dead = false;   // 用户手动切换：重新给蓝牙机会
    S.ble_fails = 0;
    S.ble_retry_at_us = 0;
    reevaluate();   // 蓝牙的开关由 reevaluate 按"模拟开 且 计划允许"统一决定
    ESP_LOGI(TAG, "simulation %s", S.sim ? "ON" : "OFF");
}

static void set_wifi(bool on)
{
    if (on == S.wifi) return;
    if (on) {
        if (!portal::start()) {
            // TCP/IP 栈一旦起过就撤不掉（1 秒定时器会持续打断浅睡），失败后重启回到干净状态
            ESP_LOGE(TAG, "WiFi start failed%s", portal::netif_started() ? " -> restart" : "");
            if (portal::netif_started()) esp_restart();
            return;
        }
        S.wifi = true;
        request_redraw(true);   // 进入二维码界面：大面积变化，清底刷新，保证二维码干净好扫
        reevaluate();
        return;
    }
    // 关 WiFi = 整机重启：lwIP 启动后它的 1 秒周期定时器无法停掉，会让模拟关闭时的
    // 浅睡每秒被叫醒一次；重启最干净。开机时刻锚点在 RTC 内存里，Uptime 不受影响；
    // 开机第一次刷屏本来就是清底刷新，正好清掉二维码。
    ESP_LOGI(TAG, "WiFi off -> restart");
    portal::stop();
    ble_hid_stop();   // 先正常断开，电脑立刻知道，重启后回连更快
    esp_restart();
}

[[noreturn]] static void shutdown(const char* why)
{
    ESP_LOGI(TAG, "shutdown: %s", why);
    board::led(true);
    ble_hid_stop();
    portal::stop();
    ui::clear_and_sleep();
    board::led(false);
    board::power_off();
}

[[noreturn]] static void factory_reset()
{
    ESP_LOGI(TAG, "factory reset");
    board::led(true);
    ble_hid_stop();
    portal::stop();
    nvs_flash_erase();   // 设置 + 蓝牙配对（NimBLE 存在 NVS）一起清
    s_uptime_anchor.magic = 0;   // 重置后 Uptime 从 0 开始
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

static void minute_tick()
{
    rtc::sync_system();
    schedule_next_tick();
    read_battery();
    S.uptime_min = uptime_minutes();
    if (S.low_count >= LOW_BATT_MINUTES) shutdown("low battery");
    if (S.wifi && (portal::idle_s() > WIFI_IDLE_OFF_S || portal::on_s() > WIFI_MAX_ON_S)) {
        ESP_LOGI(TAG, "WiFi auto off (idle %lus, on %lus)", (unsigned long)portal::idle_s(),
                 (unsigned long)portal::on_s());
        set_wifi(false);   // 不返回（重启）
    }
#if CONFIG_PM_PROFILING
    esp_pm_dump_locks(stdout);   // 调试：各功耗模式累计时间（sdkconfig 开 PM_PROFILING 才编进来）
#endif
    reevaluate();
    request_redraw(false);
    // 立即刷（不等合并窗口）：分钟对齐
    S.redraw_at_us = esp_timer_get_time();
}

static void print_stat()
{
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    int64_t to_move = S.next_move_us ? (S.next_move_us - esp_timer_get_time()) / 1000000 : -1;
    printf("STAT: time=%04d-%02d-%02d %02d:%02d:%02d rtc_valid=%d sim=%d ble=%d bonds=%d wifi=%d allow=%d "
           "batt=%dmV/%d%%/%dbars uptime=%lum next_move=%llds heap=%lu stack_free=%u\n",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, rtc::valid(), S.sim,
           S.connected, ble_hid_bond_count(), S.wifi, S.allow, S.batt_mv, S.batt_pct, S.batt_bars,
           (unsigned long)S.uptime_min, (long long)to_move, (unsigned long)esp_get_free_heap_size(),
           (unsigned)uxTaskGetStackHighWaterMark(nullptr));
    fflush(stdout);
}

static void handle(const ev_t& e)
{
    switch (e.type) {
    case EV_BTN_TOP_SHORT:   set_wifi(!S.wifi); break;
    case EV_BTN_TOP_LONG:    factory_reset();
    case EV_BTN_WHEEL_SHORT: set_sim(!S.sim); break;
    case EV_BTN_WHEEL_LONG:  shutdown("wheel long-press");
    case EV_SIM_SET:         set_sim(e.arg != 0); break;
    case EV_WIFI_OFF:        vTaskDelay(pdMS_TO_TICKS(500)); set_wifi(false); break;
    case EV_BLE_CONN:
        // 不信事件里的值（队列满丢事件会让边沿失配），直接取蓝牙模块的当前状态
        S.connected = ble_hid_connected();
        S.next_move_us = 0;
        reevaluate();
        break;
    case EV_SETTINGS_CHANGED:
        if (e.arg && ble_hid_running()) {   // 设备名变了：重启蓝牙使新名字生效
            stop_ble();
            start_ble();
        }
        S.next_move_us = 0;     // 按新间隔重新排程
        reevaluate();
        break;
    case EV_TIME_SET: {
        // 网页校时：由主循环统一写 RTC，避免和每分钟的 RTC→系统时间校正竞争。
        // Uptime 锚点跟着平移，保证 Uptime 连续。
        time_t t;
        int tz_min;
        if (!portal::take_browser_time(&t, &tz_min)) break;
        // 手机所在时区与设备不同：切换时区。RTC 里存的是本地时间，换了时区必须按新时区重写
        int old_tz = settings::tz_offset_min();
        bool tz_changed = tz_min != old_tz;
        if (tz_changed) settings::apply_tz(tz_min);   // 先只改运行时时区，RTC 写成功后才保存
        time_t now = time(nullptr);
        long long diff = (long long)t - (long long)now;
        if (tz_changed || llabs(diff) >= 2 || !rtc::valid()) {
            if (rtc::set(t)) {
                if (tz_changed) {
                    settings::set_tz_offset_min(tz_min);
                    ESP_LOGI(TAG, "time zone -> %+d min", tz_min);
                }
                S.boot_epoch += (time_t)diff;
                s_uptime_anchor.boot_epoch = S.boot_epoch;
                schedule_next_tick();
                ESP_LOGI(TAG, "time set by browser, diff=%llds", diff);
            } else {
                ESP_LOGE(TAG, "RTC write failed, time not set");
                if (tz_changed) settings::apply_tz(old_tz);   // RTC 仍是旧时区的本地时间，时区跟着退回
            }
        }
        reevaluate();
        break;
    }
    default:
        if (e.type == EV_DEBUG) {
            if (e.arg == DBG_SNAP) ui::dump_snapshot();
            else if (e.arg == DBG_STAT) print_stat();
            else if (e.arg == DBG_JIGGLE) jiggle();
            else if (e.arg == DBG_CLEAN) request_redraw(true);
            else if (e.arg == DBG_BLECYCLE && ble_hid_running()) {
                stop_ble();
                start_ble();
                print_stat();
            } else if ((e.arg & 0xFF) == DBG_SIMTEST) {
                // 关模拟 N 秒后自动打开：浅睡时串口收不到指令，靠它验证关→开整条路径
                set_sim(false);
                if (!s_simtest_timer) {
                    esp_timer_create_args_t a = {};
                    a.callback = [](void*) { ev_post(EV_SIM_SET, 1); };
                    a.name = "simtest";
                    esp_timer_create(&a, &s_simtest_timer);
                }
                esp_timer_stop(s_simtest_timer);
                esp_timer_start_once(s_simtest_timer, (uint64_t)(e.arg >> 8) * 1000000);
            } else if ((e.arg & 0xFF) == DBG_SCHEDTEST) {
                // 视同计划暂停 N 秒（只在内存里，不改设置/NVS；重复执行只是延长，重启即失效）
                if (!rtc::valid()) printf("schedtest: RTC 时间无效时周计划不生效，这里照样暂停\n");
                S.dbg_pause_until_us = esp_timer_get_time() + (int64_t)(e.arg >> 8) * 1000000;
                reevaluate();
            }
        }
        break;
    }
}

// ── 串口调试（只在蓝牙开着、没进浅睡时能收到） ─────────────────────────────
// snap [ms]  打印当前画面（可延时）   stat  打印状态   jiggle  立刻动一下
// clean 清底重刷       sim   模拟开关   wifi    WiFi 开关
// blecycle 重启蓝牙协议栈（查泄漏）    simtest N  关模拟 N 秒后自动打开
// schedtest N  视同计划暂停 N 秒（只在内存里）
static void console_task(void*)
{
    uart_driver_install(UART_NUM_0, 512, 0, 0, nullptr, 0);
    char line[32];
    int n = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;
        if (c != '\n' && c != '\r') {
            if (n < (int)sizeof(line) - 1) line[n++] = (char)c;
            continue;
        }
        line[n] = 0;
        n = 0;
        if (!strcmp(line, "snap")) ev_post(EV_DEBUG, DBG_SNAP);
        else if (!strncmp(line, "snap ", 5) && atoi(line + 5) > 0) {
            // snap <毫秒>：延时截图（配合会让设备进浅睡的指令一起发，截刷屏之后的画面）
            static esp_timer_handle_t t;
            if (!t) {
                esp_timer_create_args_t a = {};
                a.callback = [](void*) { ev_post(EV_DEBUG, DBG_SNAP); };
                a.name = "snap";
                esp_timer_create(&a, &t);
            }
            esp_timer_stop(t);
            esp_timer_start_once(t, (uint64_t)atoi(line + 5) * 1000);
        }
        else if (!strcmp(line, "stat")) ev_post(EV_DEBUG, DBG_STAT);
        else if (!strcmp(line, "jiggle")) ev_post(EV_DEBUG, DBG_JIGGLE);
        else if (!strcmp(line, "clean")) ev_post(EV_DEBUG, DBG_CLEAN);
        else if (!strcmp(line, "sim")) ev_post(EV_BTN_WHEEL_SHORT, 0);
        else if (!strcmp(line, "wifi")) ev_post(EV_BTN_TOP_SHORT, 0);
        else if (!strcmp(line, "blecycle")) ev_post(EV_DEBUG, DBG_BLECYCLE);
        else if (!strncmp(line, "simtest ", 8) && atoi(line + 8) > 0) {
            int sec = atoi(line + 8) > 86400 ? 86400 : atoi(line + 8);
            ev_post(EV_DEBUG, DBG_SIMTEST | (sec << 8));   // 秒数编进 arg，不经共享变量
        } else if (!strncmp(line, "schedtest ", 10) && atoi(line + 10) > 0) {
            int sec = atoi(line + 10) > 86400 ? 86400 : atoi(line + 10);
            ev_post(EV_DEBUG, DBG_SCHEDTEST | (sec << 8));
        }
    }
}

// ── 入口 ───────────────────────────────────────────────────────────────────

extern "C" void app_main()
{
    board::power_hold();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    s_q = xQueueCreate(16, sizeof(ev_t));
    settings::load();
    settings::apply_tz(settings::tz_offset_min());   // 必须在 rtc::init 之前：RTC 里存的是本地时间，换算 epoch 要用时区
    if (!rtc::init()) ESP_LOGE(TAG, "RTC init failed");
    if (esp_reset_reason() == ESP_RST_SW && s_uptime_anchor.magic == UPTIME_MAGIC &&
        s_uptime_anchor.boot_epoch <= time(nullptr)) {
        S.boot_epoch = s_uptime_anchor.boot_epoch;   // 关 WiFi 引起的软件重启：接着算
    } else {
        S.boot_epoch = time(nullptr);
        s_uptime_anchor.boot_epoch = S.boot_epoch;
        s_uptime_anchor.ble_restarts = 0;   // 上电时 RTC_NOINIT 内容是随机的
        s_uptime_anchor.magic = UPTIME_MAGIC;
    }
    S.uptime_min = uptime_minutes();
    schedule_next_tick();
    S.last_clean_hour = -1;
    S.batt_bars = -1;
    S.batt_pct = 100;   // 电池读不到时先按满电报给主机，免得 Windows 误报低电量

    board::init();
    portal::init_once();
    if (!ui::init()) ESP_LOGE(TAG, "display init failed");

    read_battery();
    S.sim = settings::sim_enabled();
    S.allow = settings::schedule_allows(rtc::valid());
    S.redraw_clean = true;
    redraw_now();

    esp_pm_config_t pm = {};
    pm.max_freq_mhz = 80;
    pm.min_freq_mhz = 40;
    pm.light_sleep_enable = true;
    if (esp_pm_configure(&pm) != ESP_OK) ESP_LOGW(TAG, "esp_pm_configure failed");

    reevaluate();   // 按模拟开关和周计划决定是否启动蓝牙
    xTaskCreate(console_task, "console", 3072, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "ready: sim=%d rtc_valid=%d batt=%dmV", S.sim, rtc::valid(), S.batt_mv);

    for (;;) {
        // 等到最近的一个截止时刻：下一个整分钟 / 下一次移动 / 待合并的刷屏
        int64_t now_us = esp_timer_get_time();
        int64_t wait_us = S.next_tick_us - now_us;
        if (S.next_move_us && S.next_move_us - now_us < wait_us) wait_us = S.next_move_us - now_us;
        if (S.redraw_at_us && S.redraw_at_us - now_us < wait_us) wait_us = S.redraw_at_us - now_us;
        if (S.dbg_pause_until_us && S.dbg_pause_until_us - now_us < wait_us) wait_us = S.dbg_pause_until_us - now_us;
        if (S.ble_retry_at_us && S.ble_retry_at_us - now_us < wait_us) wait_us = S.ble_retry_at_us - now_us;
        if (wait_us < 0) wait_us = 0;
        if (wait_us > 60 * 1000000LL) wait_us = 60 * 1000000LL;

        ev_t e;
        if (xQueueReceive(s_q, &e, pdMS_TO_TICKS(wait_us / 1000) + 1) == pdTRUE) {
            handle(e);
            while (xQueueReceive(s_q, &e, 0) == pdTRUE) handle(e);
        }

        if (esp_timer_get_time() >= S.next_tick_us) minute_tick();
        if (S.next_move_us && esp_timer_get_time() >= S.next_move_us) jiggle();
        if (S.ble_retry_at_us && esp_timer_get_time() >= S.ble_retry_at_us) {
            S.ble_retry_at_us = 0;
            reevaluate();   // 到点重试启动蓝牙
        }
        if (S.dbg_pause_until_us && esp_timer_get_time() >= S.dbg_pause_until_us) {
            S.dbg_pause_until_us = 0;
            reevaluate();
        }
        if (S.redraw_at_us && esp_timer_get_time() >= S.redraw_at_us) redraw_now();
    }
}
