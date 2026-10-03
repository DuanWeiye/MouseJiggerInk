// board.cpp — CoreInk 板级支持
//
// 引脚（M5Unified / 官方文档）：
//   G12 电源保持   G10 绿色 LED（低电平亮）   G35 电池分压（20k/5.1k → ×25.1/5.1）
//   G5  顶部按钮   G38 拨轮按下   G27 电源键（深睡唤醒用）
//   按键都是低电平有效；G38 是输入专用脚，靠板载上拉。
#include "board.h"
#include "events.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "board";

static constexpr gpio_num_t PIN_HOLD  = GPIO_NUM_12;
static constexpr gpio_num_t PIN_LED   = GPIO_NUM_10;
static constexpr gpio_num_t PIN_TOP   = GPIO_NUM_5;
static constexpr gpio_num_t PIN_WHEEL = GPIO_NUM_38;
static constexpr gpio_num_t PIN_PWR   = GPIO_NUM_27;

static constexpr int64_t LONG_PRESS_US  = 5000 * 1000;
static constexpr int64_t MIN_PRESS_US   = 40 * 1000;   // 短于此视为抖动
static constexpr int     POLL_MS        = 20;

// ── 按键 ────────────────────────────────────────────────────────────────────
// 平时不轮询：低电平中断（同时作为浅睡唤醒源）唤醒按键任务，任务关掉该脚中断后
// 以 20ms 轮询跟踪按下时长，全部松开后再开中断。这样待机时 CPU 能一直浅睡。
// 短按在松开时判定（才能和长按区分）；长按在按满 5 秒的那一刻触发，松开不再触发短按。

struct Btn {
    gpio_num_t pin;
    ev_type_t  short_ev;
    ev_type_t  long_ev;
    bool       down;
    bool       long_fired;
    int64_t    down_at;
};

static Btn s_btns[] = {
    { PIN_TOP,   EV_BTN_TOP_SHORT,   EV_BTN_TOP_LONG,   false, false, 0 },
    { PIN_WHEEL, EV_BTN_WHEEL_SHORT, EV_BTN_WHEEL_LONG, false, false, 0 },
};

static TaskHandle_t s_btn_task;

static void btn_isr(void* arg)
{
    gpio_intr_disable((gpio_num_t)(intptr_t)arg);
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_btn_task, &hp);
    portYIELD_FROM_ISR(hp);
}

static void btn_task(void*)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            bool any_down = false;
            int64_t now = esp_timer_get_time();
            for (auto& b : s_btns) {
                bool pressed = gpio_get_level(b.pin) == 0;
                if (pressed && !b.down) {
                    b.down = true;
                    b.long_fired = false;
                    b.down_at = now;
                } else if (pressed && b.down) {
                    if (!b.long_fired && now - b.down_at >= LONG_PRESS_US) {
                        b.long_fired = true;
                        ev_post(b.long_ev, 0);
                    }
                } else if (!pressed && b.down) {
                    b.down = false;
                    if (!b.long_fired && now - b.down_at >= MIN_PRESS_US) ev_post(b.short_ev, 0);
                }
                any_down |= b.down;
            }
            if (!any_down) break;
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        }
        for (auto& b : s_btns) gpio_intr_enable(b.pin);
    }
}

static void buttons_init()
{
    xTaskCreate(btn_task, "btn", 3072, nullptr, 5, &s_btn_task);
    gpio_install_isr_service(0);
    for (auto& b : s_btns) {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << b.pin;
        c.mode = GPIO_MODE_INPUT;
        c.pull_up_en = (b.pin < GPIO_NUM_34) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;  // 34~39 无内部上拉
        c.intr_type = GPIO_INTR_LOW_LEVEL;
        gpio_config(&c);
        gpio_wakeup_enable(b.pin, GPIO_INTR_LOW_LEVEL);
        gpio_isr_handler_add(b.pin, btn_isr, (void*)(intptr_t)b.pin);
    }
    esp_sleep_enable_gpio_wakeup();
}

// ── 电池 ADC ────────────────────────────────────────────────────────────────
// 4.2V 分压后约 0.85V，用 2.5dB 衰减（有效量程约 100~1250mV），比 12dB 在这一段更线性。
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;

static void adc_init()
{
    adc_oneshot_unit_init_cfg_t ucfg = {};
    ucfg.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));
    adc_oneshot_chan_cfg_t ccfg = {};
    ccfg.atten = ADC_ATTEN_DB_2_5;
    ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_7, &ccfg));  // GPIO35
    adc_cali_line_fitting_config_t cal = {};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_2_5;
    cal.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_line_fitting(&cal, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "ADC 校准不可用，电压读数会偏");
        s_cali = nullptr;
    }
}

namespace board {

void power_hold()
{
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << PIN_HOLD;
    c.mode = GPIO_MODE_OUTPUT;
    gpio_config(&c);
    gpio_set_level(PIN_HOLD, 1);
}

void init()
{
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << PIN_LED;
    c.mode = GPIO_MODE_OUTPUT;
    gpio_config(&c);
    led(false);
    adc_init();
    buttons_init();
}

void led(bool on) { gpio_set_level(PIN_LED, on ? 0 : 1); }

int battery_mv()
{
    constexpr int N = 32;
    int sum = 0;
    for (int i = 0; i < N; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, ADC_CHANNEL_7, &raw) != ESP_OK) return -1;
        sum += raw;
    }
    int raw = sum / N;
    int mv = 0;
    if (s_cali) {
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) return -1;
    } else {
        mv = raw * 1250 / 4095;
    }
    return mv * 251 / 51;
}

void power_off()
{
    ESP_LOGI(TAG, "power off");
    // 与 M5Unified 相同：先打几个脉冲再拉低，确保电源锁存电路释放
    for (int i = 0; i < 5; i++) {
        gpio_set_level(PIN_HOLD, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(PIN_HOLD, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    gpio_set_level(PIN_HOLD, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 还活着 = 插着 USB，断不了电：深睡，电源键唤醒
    ESP_LOGI(TAG, "still powered (USB) -> deep sleep, wake by power button");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);  // 去掉按键的浅睡 GPIO 唤醒
    rtc_gpio_pullup_en(PIN_PWR);
    esp_sleep_enable_ext0_wakeup(PIN_PWR, 0);
    esp_deep_sleep_start();
}

}  // namespace board
