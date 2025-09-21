#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

struct __packed imu_sample_v1 {
    uint8_t  ver;
    uint8_t  flags;
    uint16_t seq;
    uint32_t ts_us;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

void ble_init();

void imu_notify(const struct imu_sample_v1 *s);