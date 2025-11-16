#include "bluetooth.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// NimBLE includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLUETOOTH";

// Callback for received data
static bluetooth_rx_callback_t rx_callback = NULL;

// UUID definitions for our custom service
// Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service compatible)
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

// Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (read/write/notify)
static const ble_uuid128_t gatt_svr_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// Connection handle
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_handle;
static bool notify_enabled = false;

// Forward declarations
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_app_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_on_sync(void);

// GATT service definition
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Combined TX/RX Characteristic - for bidirectional communication
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .val_handle = &tx_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, // No more characteristics
            }
        },
    },
    {
        0, // No more services
    },
};

/**
 * @brief GATT characteristic access callback
 * Handles read/write operations on characteristics
 */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        ESP_LOGI(TAG, "Read request received");
        // Send "Hello World!" as response to read request
        const char *hello = "Hello World!";
        int rc = os_mbuf_append(ctxt->om, hello, strlen(hello));
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to append data for read response");
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        ESP_LOGI(TAG, "Sent 'Hello World!' as read response");
        return 0;
    }

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        ESP_LOGI(TAG, "Received %d bytes via BLE", OS_MBUF_PKTLEN(ctxt->om));

        // Read the received data
        uint8_t buf[256];  // Increased buffer size for more flexibility
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }

        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        ESP_LOGI(TAG, "Data received: 0x%02X (length: %d)", buf[0], len);

        // Call the registered callback if available
        if (rx_callback != NULL) {
            rx_callback(buf, len);
        } else {
            ESP_LOGW(TAG, "No RX callback registered, data ignored");
        }

        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * @brief GAP event handler
 * Handles connection/disconnection events
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
        } else {
            // Connection failed; resume advertising
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_enabled = false;

        // Connection terminated; resume advertising
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertise complete; reason=%d", event->adv_complete.reason);
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe event; cur_notify=%d, cur_indicate=%d",
                 event->subscribe.cur_notify, event->subscribe.cur_indicate);
        notify_enabled = (event->subscribe.cur_notify != 0) || (event->subscribe.cur_indicate != 0);
        ESP_LOGI(TAG, "Notifications/Indications %s", notify_enabled ? "ENABLED" : "DISABLED");
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update event; conn_handle=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        return 0;
    }

    return 0;
}

/**
 * @brief Start BLE advertising
 */
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // Set flags
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set TX power
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Set device name
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started as '%s'", BLE_DEVICE_NAME);
}

/**
 * @brief Callback when NimBLE stack is synced
 */
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack synced");

    // Start advertising
    ble_app_advertise();
}

/**
 * @brief Callback when NimBLE stack resets
 */
static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset, reason: %d", reason);
}

/**
 * @brief NimBLE host task
 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief GATT server initialization
 */
static int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/**
 * @brief Initialize Bluetooth
 */
esp_err_t bluetooth_init(bluetooth_rx_callback_t callback)
{
    int rc;
    ESP_LOGI(TAG, "Initializing Bluetooth...");

    // Store the callback
    rx_callback = callback;

    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize NimBLE
    ESP_LOGI(TAG, "Initializing NimBLE...");
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize the NimBLE host configuration
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    // Initialize GATT server
    rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize GATT server: %d", rc);
        return ESP_FAIL;
    }

    // Set device name
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name: %d", rc);
        return ESP_FAIL;
    }

    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Bluetooth initialized successfully");
    ESP_LOGI(TAG, "Device name: %s", BLE_DEVICE_NAME);

    return ESP_OK;
}

/**
 * @brief Send notification to connected client
 */
esp_err_t bluetooth_send_notification(const uint8_t *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "No device connected, cannot send notification");
        return ESP_ERR_INVALID_STATE;
    }

    if (!notify_enabled) {
        ESP_LOGW(TAG, "Notifications not enabled by client");
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for notification");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gattc_notify_custom(conn_handle, tx_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error sending notification; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %d bytes via notification", len);
    return ESP_OK;
}

/**
 * @brief Check if BLE client is connected
 */
bool bluetooth_is_connected(void)
{
    return (conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

/**
 * @brief Test Bluetooth connection by sending "Hello World" every 10 seconds
 */
void test_bluetooth_connection(void)
{
    const char *hello_msg = "Hello World!";

    while (1) {
        // Check if a device is connected
        if (bluetooth_is_connected() && notify_enabled) {
            // Send "Hello World" notification
            esp_err_t ret = bluetooth_send_notification(
                (const uint8_t *)hello_msg,
                strlen(hello_msg)
            );

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Test: Sent 'Hello World!' to connected device");
            } else {
                ESP_LOGW(TAG, "Test: Failed to send notification");
            }
        } else {
            ESP_LOGI(TAG, "Test: No device connected or notifications not enabled, waiting...");
        }

        // Wait 10 seconds before next transmission
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
