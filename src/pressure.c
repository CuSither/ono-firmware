#include <pressure.h>

#define PRESSURE_DEVICE DEVICE_DT_GET(DT_NODELABEL(lps28dfw))
static const struct device *const pres = PRESSURE_DEVICE;

float get_pressure() {
    sensor_sample_fetch(pres);

    struct sensor_value pressure_val;
    sensor_channel_get(pres, SENSOR_CHAN_PRESS, &pressure_val);
    float pressure = sensor_value_to_double(&pressure_val);
    printk("Pressure: %d\n", (int)(pressure*1000));
}

int pressure_init() {
    k_msleep(2000);
    bool rdy = device_is_ready(pres);
    printk("Barometer ready: %d\n", rdy);

    return rdy;
}