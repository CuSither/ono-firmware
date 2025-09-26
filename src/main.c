#include <imu.h>
#include <pmic.h>
#include <haptics.h>
// #include <pressure.h>
#include <ble.h>
#include <zephyr/kernel.h>

#define MSGQ_DEPTH      16

K_MEM_SLAB_DEFINE(pkt_slab, 20, MSGQ_DEPTH, 4);
K_MSGQ_DEFINE(tx_q, sizeof(struct sensor_packet *), MSGQ_DEPTH, 4);

static void notify_done(struct bt_conn *conn, void *user_data)
{
    k_mem_slab_free(&pkt_slab, user_data);
}

static void clear_mem_slab() {
    void* p;

    while (k_msgq_num_used_get(&tx_q)) {
        k_msgq_get(&tx_q, &p, K_NO_WAIT);
        k_mem_slab_free(&pkt_slab, p);
    }
}

static void tx_worker(void *d0, void *d1, void *d2)
{
    while (1) {
        // if (!current_conn) continue;
        struct sensor_packet *p;
        int err = k_msgq_get(&tx_q, &p, K_FOREVER);

        static struct bt_gatt_notify_params params;

        memset(&params, 0, sizeof(params));
        params.attr  = &ono_svc.attrs[IMU_SVC_IDX_MEAS_VAL];
        params.data  = p;
        params.len   = sizeof(*p);
        params.func  = notify_done;
        params.user_data = p;

        err = bt_gatt_notify_cb(current_conn, &params);
        if (err) {
            printk("gatt notify failed: %d\n", err);
            k_mem_slab_free(&pkt_slab, (void *)p);
            clear_mem_slab();
        }
    }

    return;
}

K_THREAD_DEFINE(bt_worker_tid, 4096, tx_worker, NULL, NULL, NULL, -10, 0, 0);

void sensor_packet_callback(struct sensor_packet* sensor_data)
{
    if (!notify_enabled || !current_conn) return;

    int alloc = k_mem_slab_num_used_get(&pkt_slab);

    if (alloc) {
        printk("Num slab allocations: %d\n", alloc);
    }

    struct sensor_packet *p;
    int err = k_mem_slab_alloc(&pkt_slab, (void **)&p, K_NO_WAIT);
    if (err != 0) {
        /* queue full: choose a policy: drop oldest/newest, or block */
        printk("Message queue slab is full, clearing slab\n");
        clear_mem_slab();
        return;
    }

    memcpy(p, sensor_data, sizeof(struct sensor_packet));

    if (k_msgq_put(&tx_q, &p, K_NO_WAIT) != 0) {
        printk("Message queue is full!\n");
        k_mem_slab_free(&pkt_slab, (void *)p);
        return;
    }
}

int main(void) {
    ble_init();

    pmic_init();
    imu_init(sensor_packet_callback);
    imu_start_streaming();
    k_msleep(1000);
    
    // haptics_init();
    // pressure_init();

    while (1) {
        // haptics_start();
        // get_pressure();
        print_battery_state();
        k_msleep(5000);
    }

    return 0;
}
