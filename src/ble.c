#include <ble.h>

#define BT_UUID_ONO_SERVICE_VAL     BT_UUID_128_ENCODE(0xc6ce15f6, 0x3fdf, 0x41e9, 0xb143, 0x3e806439085f)
#define BT_UUID_IMU_CHAR_VAL        BT_UUID_128_ENCODE(0x6d5e350c, 0x3115, 0x4498, 0xae46, 0xd2005911c1a1)

#define BT_UUID_ONO                 BT_UUID_DECLARE_128(BT_UUID_ONO_SERVICE_VAL)
#define BT_UUID_IMU_CHAR            BT_UUID_DECLARE_128(BT_UUID_IMU_CHAR_VAL)

static bool notify_enabled;

// static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) { /* ... */ }

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {

}

enum {
    IMU_SVC_IDX_PRIMARY,
    IMU_SVC_IDX_MEAS_CHRC,
    IMU_SVC_IDX_MEAS_VAL,
    IMU_SVC_IDX_MEAS_CCC,
};

BT_GATT_SERVICE_DEFINE(ono_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ONO),
	BT_GATT_CHARACTERISTIC(BT_UUID_IMU_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void imu_notify(const struct imu_sample_v1 *s)
{
    if (notify_enabled) {
        bt_gatt_notify(NULL, &ono_svc.attrs[IMU_SVC_IDX_MEAS_VAL], s, sizeof(*s));
    }
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ONO_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};


void ble_init() {
    int err = bt_enable(NULL);
    if (err) {
        printk("bt_enable failed (%d)\n", err);
        return;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (%d)\n", err);
        return;
    }

    printk("Advertising as peripheral, service in AD\n");
}