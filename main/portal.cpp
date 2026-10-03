// portal.cpp
#include "portal.h"
#include "events.h"
#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_coexist.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char* TAG = "portal";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static esp_netif_t*     s_ap_netif;
static httpd_handle_t   s_httpd;
static bool             s_running;
static portal::Status   s_status;
// 闲置超时用的时间戳存 32 位秒数：Xtensa 上 32 位读写是原子的，64 位跨任务读写会撕裂
static volatile uint32_t s_last_activity_s;   // 最近一次页面/API 请求（单调时钟，秒）
static volatile uint32_t s_started_s;         // 本次开 WiFi 的时刻（单调时钟，秒）

// 网页校时：记下收到时的单调时刻，主循环取用时补上排队耽搁的时间
static portMUX_TYPE s_time_mux = portMUX_INITIALIZER_UNLOCKED;
static time_t  s_pending_time;
static int64_t s_pending_time_at_us;
static int     s_pending_tz_min;
static bool    s_has_pending_time;

static uint32_t mono_s() { return (uint32_t)(esp_timer_get_time() / 1000000); }
static SemaphoreHandle_t s_status_lock;

// ── DNS：所有 A 记录查询都回答 192.168.4.1（强制门户） ─────────────────────
static volatile bool s_dns_run;
static volatile TaskHandle_t s_dns_task;

static void dns_task(void*)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500 * 1000 };
    if (sock < 0 || bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "dns socket/bind failed");
        if (sock >= 0) close(sock);
        s_dns_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint8_t buf[512];
    while (s_dns_run) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&from, &fl);
        if (n < 12 || (buf[2] & 0x80)) continue;     // 超时 / 太短 / 不是查询
        int p = 12;
        while (p < n && buf[p]) p += buf[p] + 1;     // 跳过 QNAME
        p += 1;
        if (p + 4 > n) continue;
        bool type_a = buf[p] == 0 && buf[p + 1] == 1;
        p += 4;                                       // QTYPE + QCLASS
        buf[2] = 0x80 | (buf[2] & 0x79) | 0x04;       // QR=1，保留 opcode 和 RD，AA=1
        buf[3] = 0x00;
        buf[6] = 0; buf[7] = type_a ? 1 : 0;          // ANCOUNT
        memset(&buf[8], 0, 4);                        // NSCOUNT / ARCOUNT（丢掉 EDNS）
        int len = p;
        if (type_a) {
            static const uint8_t ans[] = { 0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, 60, 0, 4, 192, 168, 4, 1 };
            if (len + (int)sizeof(ans) > (int)sizeof(buf)) continue;   // 超长查询直接丢弃，防越界
            memcpy(&buf[len], ans, sizeof(ans));
            len += sizeof(ans);
        }
        sendto(sock, buf, len, 0, (struct sockaddr*)&from, fl);
    }
    close(sock);
    s_dns_task = nullptr;
    vTaskDelete(nullptr);
}

// ── HTTP 工具 ──────────────────────────────────────────────────────────────
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(char* s)
{
    char* o = s;
    for (char* p = s; *p; p++) {
        if (*p == '+') *o++ = ' ';
        else if (*p == '%' && hexval(p[1]) >= 0 && hexval(p[2]) >= 0) {
            *o++ = (char)(hexval(p[1]) * 16 + hexval(p[2]));
            p += 2;
        } else *o++ = *p;
    }
    *o = 0;
}

// 读取 x-www-form-urlencoded 请求体
static bool read_body(httpd_req_t* req, char* buf, size_t n)
{
    if (req->content_len >= n) return false;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r <= 0) return false;
        got += r;
    }
    buf[got] = 0;
    return true;
}

enum FormResult { FORM_OK, FORM_MISSING, FORM_TOO_LONG };

static FormResult form_get(const char* body, const char* key, char* out, size_t n)
{
    esp_err_t e = httpd_query_key_value(body, key, out, n);
    if (e == ESP_ERR_HTTPD_RESULT_TRUNC) return FORM_TOO_LONG;
    if (e != ESP_OK) return FORM_MISSING;
    url_decode(out);
    return FORM_OK;
}

// 整数字段：先按 long 解析再夹到 [lo, hi]，不让 "70000"/"-1" 在强转时回绕。
// 空、非数字、带尾巴的输入返回 FORM_TOO_LONG 以外的错误由调用方回 400，不静默改成下限。
enum IntResult { INT_OK, INT_MISSING, INT_BAD };

static IntResult form_int(const char* body, const char* key, long lo, long hi, long* out)
{
    char v[16];
    FormResult fr = form_get(body, key, v, sizeof(v));
    if (fr == FORM_MISSING) return INT_MISSING;
    if (fr == FORM_TOO_LONG) return INT_BAD;
    char* end = nullptr;
    long x = strtol(v, &end, 10);
    if (end == v || *end != 0) return INT_BAD;
    *out = x < lo ? lo : (x > hi ? hi : x);
    return INT_OK;
}

// 只有页面和 API 请求算"有人在用"；联网检测之类的重定向请求不算，否则手机后台的
// 周期性检测会让热点永远不超时
static void touch() { s_last_activity_s = mono_s(); }

static void json_escape(const char* in, char* out, size_t n)
{
    size_t o = 0;
    for (; *in && o + 7 < n; in++) {
        unsigned char c = *in;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c < 0x20) o += snprintf(out + o, n - o, "\\u%04x", c);
        else out[o++] = c;
    }
    out[o] = 0;
}

static esp_err_t send_json(httpd_req_t* req, const char* s)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, s);
}

static esp_err_t send_ok(httpd_req_t* req) { return send_json(req, "{\"ok\":true}"); }

// ── HTTP 处理函数 ──────────────────────────────────────────────────────────
static esp_err_t h_index(httpd_req_t* req)
{
    touch();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t h_status(httpd_req_t* req)
{
    touch();
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    portal::Status st = s_status;
    xSemaphoreGive(s_status_lock);
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    char ts[24];
    strftime(ts, sizeof(ts), "%Y/%m/%d %H:%M:%S", &t);
    char j[320];
    snprintf(j, sizeof(j),
             "{\"sim\":%s,\"ble\":%s,\"bond\":%d,\"battMv\":%d,\"battPct\":%d,\"uptimeMin\":%lu,"
             "\"timeValid\":%s,\"time\":\"%s\",\"schedAllow\":%s,\"tzMin\":%d}",
             st.sim ? "true" : "false", st.ble ? "true" : "false", st.bonds, st.batt_mv, st.batt_pct,
             (unsigned long)st.uptime_min, st.time_valid ? "true" : "false", ts,
             st.sched_allow ? "true" : "false", st.tz_min);
    return send_json(req, j);
}

static esp_err_t h_get_settings(httpd_req_t* req)
{
    touch();
    Settings c = settings::get();
    char name[96], j[256];
    json_escape(c.btName, name, sizeof(name));
    snprintf(j, sizeof(j),
             "{\"btName\":\"%s\",\"moveMin\":%u,\"moveMax\":%u,\"ampMin\":%u,\"ampMax\":%u,"
             "\"wd\":\"%06lx\",\"we\":\"%06lx\"}",
             name, c.moveMinSec, c.moveMaxSec, c.ampMin, c.ampMax,
             (unsigned long)c.schedWeekday, (unsigned long)c.schedWeekend);
    return send_json(req, j);
}

static esp_err_t h_post_settings(httpd_req_t* req)
{
    touch();
    char body[512], v[160];
    if (!read_body(req, body, sizeof(body))) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    Settings c = settings::get();
    char old_name[sizeof(c.btName)];
    strlcpy(old_name, c.btName, sizeof(old_name));

    // 设备名按字节限长（中文一个字 3 字节），超长直接报错，不静默截断
    FormResult fr = form_get(body, "name", v, sizeof(v));
    if (fr == FORM_TOO_LONG || (fr == FORM_OK && strlen(v) >= sizeof(c.btName)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name too long (max 31 bytes)");
    if (fr == FORM_OK) strlcpy(c.btName, v, sizeof(c.btName));

    struct { const char* key; long lo, hi; long val; IntResult r; } nums[] = {
        { "moveMin", 5, 65535, 0, INT_MISSING }, { "moveMax", 5, 65535, 0, INT_MISSING },
        { "ampMin", 1, 100, 0, INT_MISSING },    { "ampMax", 1, 100, 0, INT_MISSING },
    };
    for (auto& n : nums) {
        n.r = form_int(body, n.key, n.lo, n.hi, &n.val);
        if (n.r == INT_BAD) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "number fields must be integers");
    }
    if (nums[0].r == INT_OK) c.moveMinSec = (uint16_t)nums[0].val;
    if (nums[1].r == INT_OK) c.moveMaxSec = (uint16_t)nums[1].val;
    if (nums[2].r == INT_OK) c.ampMin = (uint8_t)nums[2].val;
    if (nums[3].r == INT_OK) c.ampMax = (uint8_t)nums[3].val;
    if (form_get(body, "wd", v, sizeof(v)) == FORM_OK) c.schedWeekday = strtoul(v, nullptr, 16);
    if (form_get(body, "we", v, sizeof(v)) == FORM_OK) c.schedWeekend = strtoul(v, nullptr, 16);
    settings::put(c);
    bool renamed = strcmp(old_name, settings::get().btName) != 0;
    ev_post(EV_SETTINGS_CHANGED, renamed ? 1 : 0);
    return send_json(req, renamed ? "{\"ok\":true,\"renamed\":true}" : "{\"ok\":true,\"renamed\":false}");
}

static esp_err_t h_defaults(httpd_req_t* req)
{
    touch();
    Settings c = settings::get(), d;
    settings::defaults(d);
    bool renamed = strcmp(c.btName, d.btName) != 0;
    settings::put(d);
    ev_post(EV_SETTINGS_CHANGED, renamed ? 1 : 0);
    return send_ok(req);
}

// 手机浏览器打开页面时自动 POST 当前时间（epoch 秒）。这里只校验范围后投给主循环，
// 由主循环统一写 RTC（差 2 秒以上才写），不和每分钟的 RTC 校正竞争。
static esp_err_t h_time(httpd_req_t* req)
{
    touch();
    char body[64], v[24];
    if (!read_body(req, body, sizeof(body)) || form_get(body, "t", v, sizeof(v)) != FORM_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need t");
    long long t = atoll(v);
    if (t < 1700000000LL || t >= 4102444800LL)   // 2023-11 ~ 2100-01
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad t");
    long tz = settings::tz_offset_min();   // 没带 tz 就保持设备当前时区
    if (form_int(body, "tz", -720, 840, &tz) == INT_BAD)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad tz");
    taskENTER_CRITICAL(&s_time_mux);
    s_pending_tz_min = (int)tz;
    s_pending_time = (time_t)t;
    s_pending_time_at_us = esp_timer_get_time();
    s_has_pending_time = true;
    taskEXIT_CRITICAL(&s_time_mux);
    ev_post(EV_TIME_SET, 0);
    return send_ok(req);
}

static esp_err_t h_sim(httpd_req_t* req)
{
    touch();
    char body[32], v[8];
    if (!read_body(req, body, sizeof(body)) || form_get(body, "on", v, sizeof(v)) != FORM_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need on");
    ev_post(EV_SIM_SET, atoi(v) ? 1 : 0);
    return send_ok(req);
}

static esp_err_t h_wifioff(httpd_req_t* req)
{
    send_ok(req);
    ev_post(EV_WIFI_OFF, 0);   // 主循环稍后关闭（先让这个响应发出去）
    return ESP_OK;
}

// 未知路径（含各系统的联网检测 URL）一律 302 到首页，触发"登录网络"弹窗
static esp_err_t h_redirect(httpd_req_t* req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, nullptr, 0);
}

static bool start_httpd()
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 6144;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_httpd = nullptr;
        return false;
    }
    const httpd_uri_t uris[] = {
        { "/",             HTTP_GET,  h_index,         nullptr },
        { "/api/status",   HTTP_GET,  h_status,        nullptr },
        { "/api/settings", HTTP_GET,  h_get_settings,  nullptr },
        { "/api/settings", HTTP_POST, h_post_settings, nullptr },
        { "/api/defaults", HTTP_POST, h_defaults,      nullptr },
        { "/api/time",     HTTP_POST, h_time,          nullptr },
        { "/api/sim",      HTTP_POST, h_sim,           nullptr },
        { "/api/wifioff",  HTTP_POST, h_wifioff,       nullptr },
        { "/*",            HTTP_GET,  h_redirect,      nullptr },
    };
    for (const auto& u : uris) httpd_register_uri_handler(s_httpd, &u);
    return true;
}

namespace portal {

void init_once()
{
    s_status_lock = xSemaphoreCreateMutex();
}

// TCP/IP 栈推迟到第一次开 WiFi 时才起：lwIP 一旦启动，ARP/IP 重组/DNS 等 1 秒周期定时器
// 就一直跑、每秒把 CPU 从浅睡里叫醒，且无法反初始化。从没开过 WiFi 时就完全没有这份开销。
static void netif_init_lazy()
{
    if (s_ap_netif) return;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ap_netif = esp_netif_create_default_wifi_ap();
    // 这里不碰 DHCP 服务器：提前启动的话它的定时器会在 WiFi 关着时每秒把 CPU 从浅睡里叫醒
}

bool start()
{
    if (s_running) return true;
    netif_init_lazy();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&ic) != ESP_OK) return false;
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t ap = {};
    strlcpy((char*)ap.ap.ssid, AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.channel = 1;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 3;
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (esp_wifi_start() != ESP_OK) {
        esp_wifi_deinit();
        return false;
    }
    esp_coex_preference_set(ESP_COEX_PREFER_BT);   // 射频仲裁偏向蓝牙：主业是鼠标

    // DHCP option 114（RFC 8910）：直接告诉客户端门户地址，较新的系统会立即弹出
    static const char uri[] = "http://192.168.4.1/";
    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_dhcps_option(s_ap_netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, (void*)uri, strlen(uri));
    esp_netif_dhcps_start(s_ap_netif);

    s_running = true;   // 之后的失败路径统一走 stop() 清理
    s_dns_run = true;
    TaskHandle_t t = nullptr;
    if (xTaskCreate(dns_task, "dns", 3072, nullptr, 4, &t) != pdPASS) {
        s_dns_run = false;
        stop();
        return false;
    }
    s_dns_task = t;
    if (!start_httpd()) {
        stop();
        return false;
    }
    touch();
    s_started_s = mono_s();
    ESP_LOGI(TAG, "AP \"%s\" up, http://%s/", AP_SSID, AP_URL_HOST);
    return true;
}

void stop()
{
    if (!s_running) return;
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = nullptr;
    }
    s_dns_run = false;
    for (int i = 0; i < 100 && s_dns_task; i++) vTaskDelay(pdMS_TO_TICKS(10));
    esp_wifi_stop();
    esp_wifi_deinit();
    s_running = false;
    ESP_LOGI(TAG, "AP down");
}

bool running() { return s_running; }

uint32_t idle_s() { return mono_s() - s_last_activity_s; }
uint32_t on_s() { return mono_s() - s_started_s; }
bool netif_started() { return s_ap_netif != nullptr; }

bool take_browser_time(time_t* out, int* tz_min)
{
    taskENTER_CRITICAL(&s_time_mux);
    bool has = s_has_pending_time;
    time_t t = s_pending_time;
    *tz_min = s_pending_tz_min;
    int64_t at = s_pending_time_at_us;
    s_has_pending_time = false;
    taskEXIT_CRITICAL(&s_time_mux);
    if (!has) return false;
    *out = t + (time_t)((esp_timer_get_time() - at) / 1000000);   // 补上排队耽搁的时间
    return true;
}

void set_status(const Status& s)
{
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    s_status = s;
    xSemaphoreGive(s_status_lock);
}

}  // namespace portal
