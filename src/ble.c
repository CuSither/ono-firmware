#include <ble.h>

#define BT_UUID_ONO_SERVICE_VAL     BT_UUID_128_ENCODE(0xc6ce15f6, 0x3fdf, 0x41e9, 0xb143, 0x3e806439085f)
#define BT_UUID_IMU_CHAR_VAL        BT_UUID_128_ENCODE(0x6d5e350c, 0x3115, 0x4498, 0xae46, 0xd2005911c1a1)
#define BT_UUID_CAL_CHAR_VAL        BT_UUID_128_ENCODE(0xac65b5bf, 0x3afd, 0x41ae, 0x9ecd, 0x729af13a480c)

#define BT_UUID_ONO                 BT_UUID_DECLARE_128(BT_UUID_ONO_SERVICE_VAL)
#define BT_UUID_IMU_CHAR            BT_UUID_DECLARE_128(BT_UUID_IMU_CHAR_VAL)
#define BT_UUID_CAL_CHAR            BT_UUID_DECLARE_128(BT_UUID_CAL_CHAR_VAL)

#define LOG_MODULE_NAME peripheral_imu
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

bool notify_enabled = false;
struct bt_conn *current_conn = NULL;

void (*cal_cb)(bool containsData, void* buff, uint16_t len);

static struct k_work adv_work;

#define MSGQ_DEPTH      32
#define PACKET_SIZE     20

K_MEM_SLAB_DEFINE(pkt_slab, PACKET_SIZE, MSGQ_DEPTH, 4);
K_MSGQ_DEFINE(tx_q, sizeof(void*), MSGQ_DEPTH, 4);

static void notify_done(struct bt_conn *conn, void *user_data) {
    k_mem_slab_free(&pkt_slab, user_data);
}

static void clear_mem_slab() {
    void* p;

    while (k_msgq_get(&tx_q, &p, K_NO_WAIT) == 0) {
        k_mem_slab_free(&pkt_slab, p);
    }
}

static void indicate_cb(struct bt_conn *conn,
                        struct bt_gatt_indicate_params *params,
                        uint8_t err)
{
    /* Called when client acks (or times out / disconnects). */

    LOG_INF("Received indication cb, err val: %d", err);
    k_mem_slab_free(&pkt_slab, params->data);
}

static ssize_t indx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_INF("Received message from client, len %d", len);

    float* buf_f = (float*)buf;
    int8_t* buf_b = (int8_t*)buf;
    
    if (len < 4) {
        cal_cb(false, NULL, 0);
        return len;
    }

    cal_cb(true, buf, len);
    
}

static void send_sensor_packet(const void* p) {
    if (!notify_enabled) {
        k_mem_slab_free(&pkt_slab, p);
        return;
    }

    static struct bt_gatt_notify_params params;

    memset(&params, 0, sizeof(params));
    params.attr  = &ono_svc.attrs[IMU_SVC_IDX_MEAS_VAL];
    params.data  = p;
    params.len   = PACKET_SIZE;
    params.func  = notify_done;
    params.user_data = p;

    int err = bt_gatt_notify_cb(current_conn, &params);
    if (err) {
        LOG_INF("Gatt notify failed: %d", err);
        k_mem_slab_free(&pkt_slab, p);
    }
}

static void send_cal_packet(const void *p) {
    LOG_INF("Transmitting callibration packet");
    static struct bt_gatt_indicate_params ip; /* lifetime must cover the operation */
    ip.attr = &ono_svc.attrs[CAL_SVC_IDX_MEAS_VAL];;   /* required: the *value* attribute */
    ip.func = indicate_cb;       /* confirmation callback */
    ip.data = p;
    ip.len  = PACKET_SIZE;
    int err = bt_gatt_indicate(current_conn, &ip);
    if (err) {
        LOG_ERR("Gatt indicate failed: %d", err);
    }
}

static void tx_worker(void *d0, void *d1, void *d2) {
    while (1) {
        if (!current_conn) {
            k_msleep(500);
            continue;
        }

        void *p;
        k_msgq_get(&tx_q, &p, K_FOREVER);

        uint8_t *p_meta = (uint8_t*)p;
        uint8_t flags = p_meta[1];

        if (flags & BIT(0)) {
            send_cal_packet(p);
        } else {
            send_sensor_packet(p);
        }
    }
}

K_THREAD_DEFINE(bt_worker_tid, 4096, tx_worker, NULL, NULL, NULL, -5, 0, 1000);

void push_packet_to_queue(void* sensor_data) {
    if (!current_conn) return;

    LOG_DBG("num slab allocations: %d", k_mem_slab_num_used_get(&pkt_slab));

    void *p;
    int err = k_mem_slab_alloc(&pkt_slab, (void **)&p, K_NO_WAIT);
    if (err != 0) {
        LOG_INF("Message queue slab is full, clearing queue");
        clear_mem_slab();
        return;
    }

    memcpy(p, sensor_data, PACKET_SIZE);

    if (k_msgq_put(&tx_q, &p, K_NO_WAIT) != 0) {
        LOG_ERR("Message queue is full");
        k_mem_slab_free(&pkt_slab, (void *)p);
        return;
    }
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(ono_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ONO),
	BT_GATT_CHARACTERISTIC(BT_UUID_IMU_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_CAL_CHAR,
        BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, indx_write, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE) // perhaps this needs its own cc callback
);

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

    err = bt_conn_le_phy_update(current_conn, BT_CONN_LE_PHY_PARAM_2M);
    if (err) {
        LOG_ERR("Phy update request failed: %d",  err);
    }
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

static void recycled_cb(void) {
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

void ble_init(void (*cb)(bool containsData, void* buff, uint16_t len)) {
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (%d)", err);
        return;
    }

    cal_cb = cb;

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();
}