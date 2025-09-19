#include <pmic.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec wc_en_0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, en0_gpios);
static const struct gpio_dt_spec wc_en_1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, en1_gpios);

#define PMIC_DEVICE DEVICE_DT_GET(DT_NODELABEL(npm1300_pmic))
#define CHARGER_DEVICE DEVICE_DT_GET(DT_NODELABEL(npm1300_charger))
#define V_TERM_US DT_PROP(DT_NODELABEL(npm1300_charger), term_microvolt) 

static const struct device *const pmic = PMIC_DEVICE;
static const struct device *const charger = CHARGER_DEVICE;

static struct gpio_callback pmic_cb;

extern struct k_timer en_wc_timer;
static const int PROBE_INT_S = 10;

static int getBatteryMv() {
    /* Must run sensor_sample_fetch before calling this function */
    struct sensor_value battery_voltage;
    sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &battery_voltage);
    double volts = sensor_value_to_double(&battery_voltage);

    return (int)(volts*1000);
}

static void enable_wireless_charging(bool enable) {
    printk("Wireless charging enabled: %d\n", enable);

    gpio_pin_set_dt(&wc_en_0, enable);
    gpio_pin_set_dt(&wc_en_1, enable);
}

static void check_en_wc(struct k_work *work) {
    printk("Checking voltage\n");

    sensor_sample_fetch(charger);
    int mv = getBatteryMv();
    int v_term_ms = V_TERM_US / 1000;

    if (mv < v_term_ms * 0.98) {
        enable_wireless_charging(true);
        k_timer_stop(&en_wc_timer);
    }
}

K_WORK_DEFINE(en_wc_work, check_en_wc);

static void wc_timer_handler(struct k_timer *timer) {
    k_work_submit(&en_wc_work);
}

K_TIMER_DEFINE(en_wc_timer, wc_timer_handler, NULL);

static void pmic_evt_cb(const struct device *dev, struct gpio_callback *cb, uint32_t events) {
    if (events & BIT(NPM1300_EVENT_CHG_COMPLETED)) {
        printk("Charge completed\n");
        enable_wireless_charging(false);
        k_timer_start(&en_wc_timer, K_SECONDS(PROBE_INT_S), K_SECONDS(PROBE_INT_S));
    }
}

void print_battery_state() {
    if (!device_is_ready(charger)) {
        printk("Charger device is not ready\n");
        return;
    }

    sensor_sample_fetch(charger);
    
    struct sensor_value i;

    sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i);

    double amps = sensor_value_to_double(&i);
    int mv = getBatteryMv();

    printk("Pulling %d mA from battery; Battery voltage: %d mV\n\n", (int)(amps * 1000), mv);         
}

int pmic_init() {
    if (!device_is_ready(pmic)) { 
        return -ENODEV;
    }

    uint32_t mask = BIT(NPM1300_EVENT_CHG_COMPLETED);

    gpio_init_callback(&pmic_cb, pmic_evt_cb, mask);

    int err = mfd_npm1300_add_callback(pmic, &pmic_cb);

    if (err) {
        printk("add_callback failed: %d\n", err);
        return err;
    }

    gpio_pin_configure_dt(&wc_en_0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&wc_en_1, GPIO_OUTPUT_ACTIVE);

    return 0;
}