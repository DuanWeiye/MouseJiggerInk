// ble_hid.c — NimBLE 蓝牙 HID 相对鼠标（HOGP）
//
// GATT 布局与 Arduino 版（NimBLE-Arduino 的 NimBLEHIDDevice）保持一致：
//   设备信息 0x180A（厂商名、PnP ID）、电池 0x180F（电量，上报真实电池百分比）、
//   HID 0x1812（HID Information、Report Map、Control Point、Protocol Mode、
//   Input Report(ID 1) + Report Reference）。
// 沿用 ATOM 版加固 Windows 回连的做法：
//   - HID Information 标志位 = RemoteWake | NormallyConnectable；
//   - 已绑定主机连上但没加密时，外设主动发 Security Request（主机回连时不会重写
//     CCCD，NimBLE 要等链路重新加密后才从绑定记录恢复订阅，否则"已连接但鼠标不动"）；
//   - 双方都分发 ENC + ID 密钥（拿到主机 IRK，主机用随机私有地址回连时才认得出）。
// 省电：加密完成 5 秒后（主机做完服务发现）请求较长的连接间隔 + 从机延迟。
#include "ble_hid.h"
#include "events.h"

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

void ble_store_config_init(void);

static const char* TAG = "ble";

#define APPEARANCE_MOUSE 0x03C2

// 标准相对鼠标报文：Report ID 1，4 字节 = 按键, X, Y, 滚轮
static const uint8_t REPORT_MAP[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,   // Generic Desktop / Mouse / Collection(Application)
    0x85, 0x01,                           //   Report ID (1)
    0x09, 0x01, 0xA1, 0x00,               //   Pointer / Collection(Physical)
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,   //     Buttons 1..3
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x03,   //     5 位填充
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,   // X, Y, Wheel
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0,
};
// 与 Arduino 版 pnp(0x02, 0xe502, 0xa111, 0x0210) 的字节序列一致
static const uint8_t PNP_ID[7]   = { 0x02, 0xe5, 0x02, 0xa1, 0x11, 0x02, 0x10 };
static const uint8_t HID_INFO[4] = { 0x11, 0x01, 0x00, 0x03 };   // bcdHID 1.11, 国家码 0, 标志位
static const uint8_t REPORT_REF[2] = { 0x01, 0x01 };             // Report ID 1, Input
static const char    MANUFACTURER[] = "RAPOO";

enum { A_MANUF = 1, A_PNP, A_BATT, A_HID_INFO, A_REPORT_MAP, A_CTRL, A_PROTO, A_INPUT, A_REPORT_REF };

static uint16_t s_input_handle;
static uint16_t s_batt_handle;
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_running;
static volatile bool s_synced;      // host 与控制器同步完成（之后 nimble_port_stop 才可靠）
static volatile bool s_encrypted;   // 当前连接已加密：只有这时才算"电脑已连接"，HID 才可用
static uint8_t  s_batt = 100;
static uint8_t  s_proto = 1;
static uint8_t  s_report[4];
static uint8_t  s_own_addr_type;
static int      s_bonds = -1;   // -1 = 蓝牙还没启动过，未知
static char     s_name[33];
static esp_timer_handle_t s_param_timer;

static int append(struct os_mbuf* om, const void* d, uint16_t n)
{
    return os_mbuf_append(om, d, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int chr_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    switch ((int)(intptr_t)arg) {
    case A_MANUF:      return append(ctxt->om, MANUFACTURER, sizeof(MANUFACTURER) - 1);
    case A_PNP:        return append(ctxt->om, PNP_ID, sizeof(PNP_ID));
    case A_BATT:       return append(ctxt->om, &s_batt, 1);
    case A_HID_INFO:   return append(ctxt->om, HID_INFO, sizeof(HID_INFO));
    case A_REPORT_MAP: return append(ctxt->om, REPORT_MAP, sizeof(REPORT_MAP));
    case A_INPUT:      return append(ctxt->om, s_report, sizeof(s_report));
    case A_REPORT_REF: return append(ctxt->om, REPORT_REF, sizeof(REPORT_REF));
    case A_CTRL:       return 0;   // Suspend / Exit Suspend：鼠标无需处理
    case A_PROTO:
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) return append(ctxt->om, &s_proto, 1);
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1) os_mbuf_copydata(ctxt->om, 0, 1, &s_proto);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(0x2A29), .access_cb = chr_access, .arg = (void*)A_MANUF, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A50), .access_cb = chr_access, .arg = (void*)A_PNP, .flags = BLE_GATT_CHR_F_READ },
            { 0 },
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(0x2A19), .access_cb = chr_access, .arg = (void*)A_BATT,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_batt_handle },
            { 0 },
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(0x2A4A), .access_cb = chr_access, .arg = (void*)A_HID_INFO, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A4B), .access_cb = chr_access, .arg = (void*)A_REPORT_MAP, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A4C), .access_cb = chr_access, .arg = (void*)A_CTRL, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
            { .uuid = BLE_UUID16_DECLARE(0x2A4E), .access_cb = chr_access, .arg = (void*)A_PROTO,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
            { .uuid = BLE_UUID16_DECLARE(0x2A4D), .access_cb = chr_access, .arg = (void*)A_INPUT,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
              .val_handle = &s_input_handle,
              .descriptors = (struct ble_gatt_dsc_def[]) {
                  { .uuid = BLE_UUID16_DECLARE(0x2908), .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
                    .access_cb = chr_access, .arg = (void*)A_REPORT_REF },
                  { 0 },
              } },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event(struct ble_gap_event* ev, void* arg);

static void update_bond_count(void)
{
    int n = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &n) == 0) s_bonds = n;
}

// 未连接时广播：先快速广播 30 秒（20~30ms，便于主机尽快发现/回连），之后转慢速（约 420ms）省电
static void adv_start(bool fast)
{
    struct ble_hs_adv_fields f = { 0 };
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.appearance = APPEARANCE_MOUSE;
    f.appearance_is_present = 1;
    static ble_uuid16_t hid_uuid = BLE_UUID16_INIT(0x1812);
    f.uuids16 = &hid_uuid;
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }

    struct ble_hs_adv_fields r = { 0 };
    size_t n = strlen(s_name), cut = n;
    if (cut > 29) {   // 扫描响应最多 31 字节，名字字段头占 2；截断时退到 UTF-8 字符边界
        cut = 29;
        while (cut > 0 && ((uint8_t)s_name[cut] & 0xC0) == 0x80) cut--;
    }
    r.name = (uint8_t*)s_name;
    r.name_len = cut;
    r.name_is_complete = cut == n;
    rc = ble_gap_adv_rsp_set_fields(&r);
    if (rc) { ESP_LOGE(TAG, "adv_rsp_set_fields rc=%d", rc); return; }

    struct ble_gap_adv_params p = { 0 };
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = fast ? 32 : 668;   // 单位 0.625ms：20ms / 417.5ms
    p.itvl_max = fast ? 48 : 700;   //                30ms / 437.5ms
    rc = ble_gap_adv_start(s_own_addr_type, NULL, fast ? 30000 : BLE_HS_FOREVER, &p, gap_event, NULL);
    if (rc && rc != BLE_HS_EALREADY) ESP_LOGE(TAG, "adv_start rc=%d", rc);
    else ESP_LOGI(TAG, "advertising (%s)", fast ? "fast" : "slow");
}

static bool peer_bonded(const ble_addr_t* a)
{
    struct ble_store_key_sec k = { 0 };
    struct ble_store_value_sec v;
    k.peer_addr = *a;
    return ble_store_read_peer_sec(&k, &v) == 0;
}

static void request_low_power_params(void* arg)
{
    uint16_t c = s_conn;
    if (!s_running || c == BLE_HS_CONN_HANDLE_NONE) return;
    // 间隔 45~60ms、从机延迟 16（无数据时最多连续跳过 16 个连接事件 ≈ 1s 才收发一次）、
    // 监督超时 5s（须 > (1+延迟)×间隔×2 ≈ 2s）。主机可能不接受，结果见 CONN_UPDATE 日志。
    struct ble_gap_upd_params p = {
        .itvl_min = 36, .itvl_max = 48, .latency = 16, .supervision_timeout = 500,
        .min_ce_len = 0, .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(c, &p);
    ESP_LOGI(TAG, "request conn params rc=%d", rc);
}

static void log_conn(uint16_t handle, const char* what)
{
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(handle, &d) != 0) return;
    ESP_LOGI(TAG, "%s: peer=%02x:%02x:%02x:%02x:%02x:%02x itvl=%.2fms latency=%d timeout=%dms enc=%d bonded=%d",
             what, d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
             d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0],
             d.conn_itvl * 1.25, d.conn_latency, d.supervision_timeout * 10,
             d.sec_state.encrypted, d.sec_state.bonded);
}

static int gap_event(struct ble_gap_event* ev, void* arg)
{
    struct ble_gap_conn_desc d;
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status != 0) {
            ESP_LOGW(TAG, "connect failed status=%d", ev->connect.status);
            if (s_running) adv_start(true);
            break;
        }
        s_conn = ev->connect.conn_handle;
        s_encrypted = false;
        log_conn(s_conn, "connected");
        if (ble_gap_conn_find(s_conn, &d) == 0 && !d.sec_state.encrypted && peer_bonded(&d.peer_id_addr)) {
            ble_gap_security_initiate(s_conn);
        }
        // 这里不报"已连接"：要等加密完成（ENC_CHANGE），没配对的设备连上来不算
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected reason=0x%x", ev->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_encrypted = false;
        esp_timer_stop(s_param_timer);
        ev_post(EV_BLE_CONN, 0);
        if (s_running) adv_start(true);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_running && s_conn == BLE_HS_CONN_HANDLE_NONE && ev->adv_complete.reason == BLE_HS_ETIMEOUT) {
            adv_start(false);
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption status=%d", ev->enc_change.status);
        if (ev->enc_change.status == 0) {
            update_bond_count();
            log_conn(ev->enc_change.conn_handle, "encrypted");
            if (!s_encrypted) {
                s_encrypted = true;
                ev_post(EV_BLE_CONN, 1);
            }
            esp_timer_stop(s_param_timer);
            esp_timer_start_once(s_param_timer, 5 * 1000 * 1000);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        // 主机丢了配对又来重新配：删掉旧记录，让它重配
        if (ble_gap_conn_find(ev->repeat_pairing.conn_handle, &d) == 0) ble_store_util_delete_peer(&d.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe attr=%d notify=%d (input=%d batt=%d)", ev->subscribe.attr_handle,
                 ev->subscribe.cur_notify, s_input_handle, s_batt_handle);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        log_conn(ev->conn_update.conn_handle, ev->conn_update.status == 0 ? "params updated" : "params update failed");
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", ev->mtu.value);
        break;
    }
    return 0;
}


static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    update_bond_count();
    s_synced = true;
    ESP_LOGI(TAG, "synced, bonds=%d", s_bonds);
    adv_start(true);
}

static void on_reset(int reason)
{
    s_synced = false;
    ESP_LOGW(TAG, "host reset reason=%d", reason);
}

static void host_task(void* param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_hid_start(const char* name, uint8_t battery_pct)
{
    if (s_running) return ESP_OK;
    strlcpy(s_name, name, sizeof(s_name));
    s_batt = battery_pct;
    if (!s_param_timer) {
        const esp_timer_create_args_t a = { .callback = request_low_power_params, .name = "ble_param" };
        esp_timer_create(&a, &s_param_timer);
    }

    // 注：第二次及以后 start 时 NimBLE 会打一条 "failed to configure restored IRK"，无害——
    // ESP32 用主机侧地址解析，解析列表是静态数组、deinit 不清空，条目本来就还在，重复添加被拒而已。
    // 前提：本机地址保持 public。不要在这套反复启停的设计下启用本机 RPA/隐私——
    // ble_hs_resolv_init 只执行一次，第二次 start 后会对已 deinit 的 callout 操作。
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_svcs);
    if (rc) {
        ESP_LOGE(TAG, "gatts cfg rc=%d", rc);
        nimble_port_deinit();
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set(s_name);
    ble_svc_gap_device_appearance_set(APPEARANCE_MOUSE);
    ble_store_config_init();

    s_running = true;
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "started as \"%s\"", s_name);
    return ESP_OK;
}

void ble_hid_stop(void)
{
    if (!s_running) return;
    s_running = false;   // 先置位：断开回调里就不会再重新广播
    esp_timer_stop(s_param_timer);
    vTaskDelay(pdMS_TO_TICKS(20));   // esp_timer_stop 不等正在执行的回调；回调很短，稍等即可
    // 刚 start 还没和控制器同步时 nimble_port_stop 会失败，先等同步（最多 2 秒）
    for (int i = 0; i < 200 && !s_synced; i++) vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t c = s_conn;
    if (c != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(c, BLE_ERR_REM_USER_CONN_TERM);
        for (int i = 0; i < 50 && s_conn != BLE_HS_CONN_HANDLE_NONE; i++) vTaskDelay(pdMS_TO_TICKS(10));
    }
    ble_gap_adv_stop();
    int rc = nimble_port_stop();
    if (rc != 0) {
        // host 任务还活着时不能 deinit（会拆掉它正在用的队列/控制器）。这种半停状态没法
        // 安全恢复，直接重启：模拟开关存在 NVS，Uptime 锚点在 RTC 内存，都不受影响。
        ESP_LOGE(TAG, "nimble_port_stop rc=%d, restarting", rc);
        esp_restart();
    }
    nimble_port_deinit();
    s_synced = false;
    s_encrypted = false;
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        ev_post(EV_BLE_CONN, 0);
    }
    ESP_LOGI(TAG, "stopped");
}

bool ble_hid_running(void) { return s_running; }
bool ble_hid_connected(void) { return s_running && s_conn != BLE_HS_CONN_HANDLE_NONE && s_encrypted; }
int ble_hid_bond_count(void) { return s_bonds; }

bool ble_hid_move(int8_t dx, int8_t dy)
{
    uint16_t c = s_conn;
    if (!s_running || c == BLE_HS_CONN_HANDLE_NONE || !s_encrypted) return false;
    s_report[0] = 0;
    s_report[1] = (uint8_t)dx;
    s_report[2] = (uint8_t)dy;
    s_report[3] = 0;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(s_report, sizeof(s_report));
    if (!om) return false;
    int rc = ble_gatts_notify_custom(c, s_input_handle, om);
    if (rc) ESP_LOGW(TAG, "notify rc=%d", rc);
    return rc == 0;
}

void ble_hid_set_battery(uint8_t pct)
{
    if (pct == s_batt) return;
    s_batt = pct;
    if (s_running) ble_gatts_chr_updated(s_batt_handle);
}
