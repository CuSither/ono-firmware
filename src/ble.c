#include <ble.h>

#define BT_UUID_ONO_SERVICE_VAL     BT_UUID_128_ENCODE(0xc6ce15f6, 0x3fdf, 0x41e9, 0xb143, 0x3e806439085f)
#define BT_UUID_IMU_CHAR_VAL        BT_UUID_128_ENCODE(0x6d5e350c, 0x3115, 0x4498, 0xae46, 0xd2005911c1a1)

#define BT_UUID_ONO                 BT_UUID_DECLARE_128(BT_UUID_ONO_SERVICE_VAL)
#define BT_UUID_IMU_CHAR            BT_UUID_DECLARE_128(BT_UUID_IMU_CHAR_VAL)

#define LOG_MODULE_NAME peripheral_imu
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

bool notify_enabled = false;
struct bt_conn *current_conn = NULL;

static struct k_work adv_work;


static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_enabled = (value & BT_GATT_CCC_NOTIFY) != 0;
    // notify_enabled = true;
}

BT_GATT_SERVICE_DEFINE(ono_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ONO),
	BT_GATT_CHARACTERISTIC(BT_UUID_IMU_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// void imu_notify(const void *s, uint16_t len)
// {
//     if (notify_enabled) {
//         int ret = bt_gatt_notify(NULL, &ono_svc.attrs[IMU_SVC_IDX_MEAS_VAL], s, len);
//         if (ret) {
//             printk("Notify failed!!\n");
//         }
//     }
// }

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ONO_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static void advertising_start(void) {
	k_work_submit(&adv_work);
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
};

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}



void ble_init() {
    int err = bt_enable(NULL);
    if (err) {
        printk("bt_enable failed (%d)\n", err);
        return;
    }

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();


    // err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    // if (err) {
    //     printk("Advertising failed to start (%d)\n", err);
    //     return;
    // }

    // printk("Advertising as peripheral, service in AD\n");
}