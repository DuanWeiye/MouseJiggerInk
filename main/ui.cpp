// ui.cpp — 用 M5Canvas（1bpp 离屏画布）画界面，交给 epd 显示
#include "ui.h"
#include "epd.h"
#include "portal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <M5GFX.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "ui";

static M5Canvas s_cv;
static uint8_t  s_fb[epd::FB_SIZE];      // 转成面板格式后的帧
static uint8_t  s_last[epd::FB_SIZE];    // 上次真正刷到面板上的帧
static bool     s_has_last;
static bool     s_invert_bits;           // 画布 bit=1 是黑色时为 true（init 时自检）
static bool     s_ok;                    // 画布与面板都初始化成功

// ── 绘图小工具 ──────────────────────────────────────────────────────────────

// 粗线：两个三角形拼成的矩形 + 两端圆头（拐角处自然衔接）
static void thick_line(int x0, int y0, int x1, int y1, int w)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1) return;
    float nx = -dy / len * w / 2.0f, ny = dx / len * w / 2.0f;
    int ax = lroundf(x0 + nx), ay = lroundf(y0 + ny);
    int bx = lroundf(x0 - nx), by = lroundf(y0 - ny);
    int cx = lroundf(x1 - nx), cy = lroundf(y1 - ny);
    int ex = lroundf(x1 + nx), ey = lroundf(y1 + ny);
    s_cv.fillTriangle(ax, ay, bx, by, cx, cy, TFT_BLACK);
    s_cv.fillTriangle(ax, ay, cx, cy, ex, ey, TFT_BLACK);
    s_cv.fillCircle(x0, y0, w / 2, TFT_BLACK);
    s_cv.fillCircle(x1, y1, w / 2, TFT_BLACK);
}

// ── 各区域 ─────────────────────────────────────────────────────────────────

static constexpr int TOP_H   = 38;    // 上方状态栏高度（分隔线 y）
static constexpr int BOT_Y   = 162;   // 下方分隔线 y
static constexpr int MID_CY  = (TOP_H + BOT_Y) / 2;

static void draw_top(const ui::Model& m)
{
    char buf[16];
    if (m.batt_mv > 0) snprintf(buf, sizeof(buf), "%d.%02dV", m.batt_mv / 1000, (m.batt_mv % 1000) / 10);
    else snprintf(buf, sizeof(buf), "--V");
    s_cv.setFont(&fonts::FreeSansBold12pt7b);
    s_cv.setTextColor(TFT_BLACK);
    s_cv.setTextDatum(middle_left);
    s_cv.drawString(buf, 6, TOP_H / 2);

    // 电池外框 36x22 + 正极凸起（右边缘对齐 x=194）；内部 5 格，每格 4 宽、间距 2
    const int bw = 36, bh = 22, bx = 194 - 4 - bw, by = (TOP_H - bh) / 2;
    s_cv.fillRoundRect(bx, by, bw, bh, 3, TFT_BLACK);
    s_cv.fillRect(bx + 2, by + 2, bw - 4, bh - 4, TFT_WHITE);
    s_cv.fillRect(bx + bw, by + 6, 4, bh - 12, TFT_BLACK);
    for (int i = 0; i < 5; i++) {
        if (i < m.batt_bars) s_cv.fillRect(bx + 4 + i * 6, by + 4, 4, bh - 8, TFT_BLACK);
    }
    s_cv.fillRect(0, TOP_H, epd::W, 2, TFT_BLACK);
}

static void draw_mouse(int cx, int cy)
{
    // 机身：圆角矩形外框 52x92，线宽 5；左右键分割线；滚轮
    const int w = 52, h = 92, r = 26, t = 5;
    int x = cx - w / 2, y = cy - h / 2;
    s_cv.fillRoundRect(x, y, w, h, r, TFT_BLACK);
    s_cv.fillRoundRect(x + t, y + t, w - 2 * t, h - 2 * t, r - t, TFT_WHITE);
    int split = y + 36;
    s_cv.fillRect(x, split - 2, w, 4, TFT_BLACK);
    s_cv.fillRect(cx - 2, y, 4, split - y, TFT_BLACK);
    s_cv.fillRoundRect(cx - 4, y + 11, 8, 15, 4, TFT_BLACK);
}

static void draw_bluetooth(int cx, int cy)
{
    // ᛒ 形：竖线 + 上下两个三角箭头
    const int hh = 43, dx = 22, dy = 22, w = 7;
    int tx = cx, ty = cy - hh, bx = cx, byy = cy + hh;
    thick_line(tx, ty, bx, byy, w);
    thick_line(tx, ty, cx + dx, ty + dy, w);
    thick_line(cx + dx, ty + dy, cx - dx, byy - dy, w);
    thick_line(cx - dx, ty + dy, cx + dx, byy - dy, w);
    thick_line(cx + dx, byy - dy, bx, byy, w);
}

static void draw_cross(int cx, int cy)
{
    const int d = 34, w = 12;
    thick_line(cx - d, cy - d, cx + d, cy + d, w);
    thick_line(cx + d, cy - d, cx - d, cy + d, w);
}

static void draw_center(ui::Center c)
{
    switch (c) {
    case ui::Center::SimOff: draw_cross(100, MID_CY); break;
    case ui::Center::NoConn: draw_bluetooth(100, MID_CY); break;
    case ui::Center::Active: draw_mouse(100, MID_CY); break;
    case ui::Center::Paused:
        draw_mouse(88, MID_CY);
        s_cv.setTextColor(TFT_BLACK);
        s_cv.setTextDatum(middle_center);
        s_cv.setFont(&fonts::FreeSansBold18pt7b);
        s_cv.drawString("Z", 142, MID_CY - 26);
        s_cv.setFont(&fonts::FreeSansBold12pt7b);
        s_cv.drawString("z", 160, MID_CY - 2);
        break;
    }
}

static void draw_bottom_text(const char* s)
{
    s_cv.fillRect(0, BOT_Y, epd::W, 2, TFT_BLACK);
    s_cv.setFont(&fonts::FreeSans12pt7b);
    s_cv.setTextColor(TFT_BLACK);
    s_cv.setTextDatum(middle_center);
    s_cv.drawString(s, epd::W / 2, (BOT_Y + epd::H) / 2);
}

// "Uptime 3h 6m"；满一天后可能超出屏宽，依次换成 "Up …" 和不带前缀
static void format_uptime(uint32_t min, char* buf, size_t n)
{
    uint32_t d = min / 1440, h = (min / 60) % 24, m = min % 60;
    char v[24];
    if (d)      snprintf(v, sizeof(v), "%lud %luh %lum", (unsigned long)d, (unsigned long)h, (unsigned long)m);
    else if (h) snprintf(v, sizeof(v), "%luh %lum", (unsigned long)h, (unsigned long)m);
    else        snprintf(v, sizeof(v), "%lum", (unsigned long)m);
    s_cv.setFont(&fonts::FreeSans12pt7b);
    for (const char* prefix : { "Uptime ", "Up ", "" }) {
        snprintf(buf, n, "%s%s", prefix, v);
        if (s_cv.textWidth(buf) <= epd::W - 8) return;
    }
}

static void render(const ui::Model& m)
{
    s_cv.fillScreen(TFT_WHITE);
    draw_top(m);
    if (m.wifi) {
        // 扫码直接加入开放热点；二维码 v2(25 模块) × 4px ≈ 100px，放在中间区域
        static char qr[64];
        snprintf(qr, sizeof(qr), "WIFI:T:nopass;S:%s;;", portal::AP_SSID);
        const int size = 116;
        s_cv.qrcode(qr, (epd::W - size) / 2, MID_CY - size / 2, size, 2);
        draw_bottom_text(portal::AP_URL_HOST);
    } else {
        draw_center(m.center);
        char buf[32];
        format_uptime(m.uptime_min, buf, sizeof(buf));
        draw_bottom_text(buf);
    }
}

// 画布 → 面板格式（bit=1 白、高位在左）
static void canvas_to_fb()
{
    const uint8_t* src = (const uint8_t*)s_cv.getBuffer();
    for (int i = 0; i < epd::FB_SIZE; i++) s_fb[i] = s_invert_bits ? ~src[i] : src[i];
}

namespace ui {

bool init()
{
    s_cv.setColorDepth(1);
    if (!s_cv.createSprite(epd::W, epd::H)) {
        ESP_LOGE(TAG, "画布分配失败");
        return false;
    }
    // 自检画布位格式：全白后在 (0,0) 画一个黑点，期望首字节 0x7F（bit=1 白、高位在左）
    s_cv.fillScreen(TFT_WHITE);
    s_cv.drawPixel(0, 0, TFT_BLACK);
    uint8_t b0 = ((const uint8_t*)s_cv.getBuffer())[0];
    if (b0 == 0x7F) s_invert_bits = false;
    else if (b0 == 0x80) s_invert_bits = true;
    else {
        ESP_LOGE(TAG, "画布位格式不认识: 0x%02X", b0);
        return false;
    }
    ESP_LOGI(TAG, "canvas ok (first byte 0x%02X, invert=%d)", b0, s_invert_bits);
    s_ok = epd::init();
    return s_ok;
}

bool show(const Model& m, bool clean)
{
    if (!s_ok) return false;   // 显示坏了也让其它功能照常工作
    render(m);
    canvas_to_fb();
    if (!clean && s_has_last && memcmp(s_fb, s_last, sizeof(s_fb)) == 0) return false;
    epd::present(s_fb, clean || !s_has_last);
    memcpy(s_last, s_fb, sizeof(s_fb));
    s_has_last = true;
    return true;
}

void clear_and_sleep()
{
    if (!s_ok) return;
    memset(s_fb, 0xFF, sizeof(s_fb));
    epd::present(s_fb, true);
    epd::sleep();
}

void dump_snapshot()
{
    static const char HEX[] = "0123456789ABCDEF";
    char line[129];
    printf("SNAP:");
    for (int i = 0; i < epd::FB_SIZE; i += 64) {
        int n = 0;
        for (int j = i; j < i + 64 && j < epd::FB_SIZE; j++) {
            line[n++] = HEX[s_last[j] >> 4];
            line[n++] = HEX[s_last[j] & 15];
        }
        line[n] = 0;
        fputs(line, stdout);
    }
    printf("\n");
    fflush(stdout);
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(1000));   // 发完再返回，否则剩下的字节要等下次唤醒（最长约 1 分钟）才发出
}

}  // namespace ui
