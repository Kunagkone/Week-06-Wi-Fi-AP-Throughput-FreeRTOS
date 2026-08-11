/* ============================================================
 * Lab 6.2 - Node B: ESP32 Station Client Profiler
 * ============================================================
 * หน้าที่:
 *   1. เชื่อมต่อไปยัง Node A (SoftAP) แบบ Station
 *   2. ส่งข้อมูล Benchmark Payload ขนาด 50 KB จำนวน 10 รอบ
 *   3. แต่ละรอบ: จับเวลา -> คำนวณ Throughput (Kbps)
 *              -> อ่านค่า RSSI ปัจจุบันจาก esp_wifi_sta_get_ap_info()
 *   4. พิมพ์สรุปผลแต่ละรอบผ่าน Serial Monitor
 *
 * วิธีใช้คู่กับ Node A:
 *   - Node A จะเป็นฝ่ายปรับ Tx Power (TX_POWER_QUARTER_DBM) ทีละระดับ
 *     แล้ว build+flash ใหม่ก่อนแต่ละรอบทดลอง
 *   - Node B ไฟล์นี้ไม่ต้องแก้อะไร แค่รันซ้ำทุกครั้งที่ Node A
 *     เปลี่ยน Tx Power แล้ว (กด Reset บอร์ด Node B ใหม่ หรือปล่อยให้
 *     retry auto-reconnect เอง)
 * ============================================================ */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "CLIENT_PROFILER";

/* ---------------- แก้ให้ตรงกับ Node A ของคู่ทดลอง ---------------- */
#define TARGET_WIFI_SSID        "ESP32_AP_0052"   // ต้องตรงกับ SSID ของ Node A
#define TARGET_WIFI_PASS        "67030052"
#define SERVER_IP                "192.168.4.1"
#define SERVER_PORT              8080

#define PAYLOAD_SIZE_BYTES      (50 * 1024)   // 50 KB ต่อรอบ
#define CHUNK_SIZE               1024
#define TOTAL_ROUNDS              10
#define WIFI_MAXIMUM_RETRY       10

static int s_retry_num = 0;
static bool s_wifi_connected = false;

/* ============================================================
 * Wi-Fi Event Handler
 * ============================================================ */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "[FORENSIC EVENT]: Station started; connecting to %s", TARGET_WIFI_SSID);
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        s_wifi_connected = false;
        ESP_LOGW(TAG, "[FORENSIC EVENT]: Disconnected, reason=%d", event->reason);

        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d attempts", WIFI_MAXIMUM_RETRY);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "[FORENSIC EVENT]: Connected; IP=" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_wifi_connected = true;
    }
}

/* ============================================================
 * อ่านค่า RSSI ปัจจุบันจาก AP ที่กำลังเชื่อมต่ออยู่
 * ============================================================ */
static int8_t read_current_rssi(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0; // อ่านไม่ได้ (ยังไม่เชื่อมต่อ)
}

/* ============================================================
 * ส่ง Payload 50KB ไปยัง Node A ผ่าน TCP แล้ววัดเวลา/ความเร็ว
 * ============================================================ */
static bool run_benchmark_round(int round_num, float *out_seconds, float *out_kbps, int8_t *out_rssi)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed, errno=%d", errno);
        return false;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(SERVER_PORT),
    };
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    ESP_LOGI(TAG, "[ROUND %d/%d]: Connecting to %s:%d",
             round_num, TOTAL_ROUNDS, SERVER_IP, SERVER_PORT);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "connect() failed, errno=%d", errno);
        close(sock);
        return false;
    }

    static uint8_t payload_chunk[CHUNK_SIZE];
    memset(payload_chunk, 0xAB, sizeof(payload_chunk));   // ข้อมูล dummy

    int64_t t_start = esp_timer_get_time();   // เวลาเริ่ม (microseconds)

    int total_sent = 0;
    while (total_sent < PAYLOAD_SIZE_BYTES) {
        int to_send = PAYLOAD_SIZE_BYTES - total_sent;
        if (to_send > CHUNK_SIZE) to_send = CHUNK_SIZE;

        int sent = send(sock, payload_chunk, to_send, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "send() failed, errno=%d", errno);
            close(sock);
            return false;
        }
        total_sent += sent;
    }

    /* ปิดฝั่งส่ง เพื่อให้ Node A รู้ว่าข้อมูลหมดแล้ว (recv จะ return 0) */
    shutdown(sock, SHUT_WR);

    /* รอ ACK กลับจาก Node A (ใช้ยืนยันว่าฝั่งรับ ประมวลผลจบจริง) */
    char ack_buf[16];
    recv(sock, ack_buf, sizeof(ack_buf), 0);

    int64_t t_end = esp_timer_get_time();     // เวลาสิ้นสุด (microseconds)
    close(sock);

    float seconds = (t_end - t_start) / 1000000.0f;
    float kbps = (PAYLOAD_SIZE_BYTES * 8.0f / 1000.0f) / seconds;  // kilobits per second
    int8_t rssi = read_current_rssi();

    *out_seconds = seconds;
    *out_kbps    = kbps;
    *out_rssi    = rssi;

    return true;
}

/* ============================================================
 * Benchmark Task - รอ Wi-Fi เชื่อมต่อสำเร็จ แล้ววนรัน 10 รอบ
 * ============================================================ */
static void benchmark_task(void *arg)
{
    ESP_LOGI(TAG, "Client profiler ready: 50 KB x %d rounds", TOTAL_ROUNDS);

    /* รอจนกว่า Wi-Fi จะเชื่อมต่อสำเร็จและได้ IP แล้ว */
    while (!s_wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    for (int round = 1; round <= TOTAL_ROUNDS; round++) {
        float seconds = 0, kbps = 0;
        int8_t rssi = 0;

        bool ok = run_benchmark_round(round, &seconds, &kbps, &rssi);

        if (ok) {
            ESP_LOGI(TAG, "=======================================================");
            ESP_LOGI(TAG, " [BENCHMARK RESULT %d/%d]", round, TOTAL_ROUNDS);
            ESP_LOGI(TAG, "  -> Current RSSI       : %d dBm", rssi);
            ESP_LOGI(TAG, "  -> Total Transferred  : %d Bytes", PAYLOAD_SIZE_BYTES);
            ESP_LOGI(TAG, "  -> Time Elapsed       : %.3f Seconds", seconds);
            ESP_LOGI(TAG, "  -> Measured Speed     : %.2f Kbps", kbps);
            ESP_LOGI(TAG, "=======================================================");
        } else {
            ESP_LOGW(TAG, "[ROUND %d/%d]: Failed, retrying next round", round, TOTAL_ROUNDS);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // เว้นจังหวะระหว่างรอบ
    }

    ESP_LOGI(TAG, "All benchmark rounds completed");
    vTaskDelete(NULL);
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "[FORENSIC]: Call nvs_flash_init()");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = TARGET_WIFI_SSID,
            .password = TARGET_WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(benchmark_task, "benchmark_task", 4096, NULL, 5, NULL);
}