#include <zephyr/kernel.h>
// #include <zephyr/arch/arm64/arch.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <math.h>
#include "Fusion.h"

#include <zephyr/sys/byteorder.h>

struct __packed sensor_packet {
    uint8_t v;
    uint8_t flags;
    uint32_t time_stamp_ns;
    int16_t temp;
    int16_t aX, aY, aZ;
    int16_t gX, gY, gZ;
};

struct __packed cal_data_packet {
    uint8_t v;
    uint8_t flags;
    float ax, ay, az;
    uint8_t filler[6];
};

int imu_init(void (*transmit_cb)(struct sensor_packet*));
void start_callibration();
void feed_callibration_data(void* buff, uint16_t len);
int imu_start_streaming();