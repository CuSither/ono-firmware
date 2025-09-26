#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/hci.h>

struct __packed imu_sample_v1 {
    uint8_t  ver;
    uint8_t  flags;
    uint16_t seq;
    uint32_t ts_us;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

extern bool notify_enabled;
extern struct bt_conn *current_conn;

enum {
    IMU_SVC_IDX_PRIMARY,
    IMU_SVC_IDX_MEAS_CHRC,
    IMU_SVC_IDX_MEAS_VAL,
    IMU_SVC_IDX_MEAS_CCC,
};

extern const struct bt_gatt_service_static ono_svc;

void ble_init();

void push_packet_to_queue(void* sensor_data);