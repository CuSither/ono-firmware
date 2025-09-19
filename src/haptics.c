#include <haptics.h>

#define HAPTICS_DEVICE DEVICE_DT_GET(DT_NODELABEL(drv_2605))
static const struct device *const vibe = HAPTICS_DEVICE;

static const uint32_t hold = 5000000;
static const uint8_t amplitude = 127;

static const struct drv2605_rtp_data config_data = {
    .size=1,
    .rtp_hold_us=&hold,
    .rtp_input=&amplitude
};

static union drv2605_config_data config = { .rtp_data = &config_data };

int haptics_init() {
    if (!device_is_ready(vibe)) {
        return -ENODEV;
    }

    int err = drv2605_haptic_config(vibe, DRV2605_HAPTICS_SOURCE_RTP, &config);

    return err;
}

int haptics_start() {
    return haptics_start_output(vibe);
}