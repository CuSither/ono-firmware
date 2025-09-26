#include <imu.h>

#define STREAMDEV_ALIAS DT_ALIAS(imu)
#define STREAMDEV_DEVICE DEVICE_DT_GET(STREAMDEV_ALIAS)
static const struct device *const imu = STREAMDEV_DEVICE;

SENSOR_DT_STREAM_IODEV(iodev, STREAMDEV_ALIAS,
    {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE},
    {SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_NOP});

/* ToDo: Calculate how much memory is actually needed */
RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, 4, 4, 128, 16, 4);

struct sensor_chan_spec temp_chan = { SENSOR_CHAN_DIE_TEMP, 0 };
struct sensor_chan_spec accel_chan = { SENSOR_CHAN_ACCEL_XYZ, 0 };
struct sensor_chan_spec gyro_chan = { SENSOR_CHAN_GYRO_XYZ, 0 };

// #define TASK_STACK_SIZE           8192ul
// K_THREAD_STACK_DEFINE(thread_stack, TASK_STACK_SIZE);
// static struct k_thread thread_id;

void (*ble_transmit_cb)(struct sensor_packet*);

struct __packed icm42688_packet {
    uint8_t header;

    int16_t ac_x;
    int16_t ac_y;
    int16_t ac_z;

    int16_t gy_x;
    int16_t gy_y;
    int16_t gy_z;

    int8_t temp;

    uint16_t ts;
};

struct icm42688_decoder_header {
	uint64_t timestamp;
	uint8_t is_fifo: 1;
	uint8_t gyro_fs: 3;
	uint8_t accel_fs: 2;
	uint8_t reserved: 2;
} __attribute__((__packed__));

struct icm42688_fifo_data {
	struct icm42688_decoder_header header;
	uint8_t int_status;
	uint16_t gyro_odr: 4;
	uint16_t accel_odr: 4;
	uint16_t fifo_count: 11;
	uint16_t reserved: 5;
} __attribute__((__packed__));



static void stream_data(void *p1, void *p2, void *p3) {
    // const struct device *dev = (const struct device *)p1;
    struct rtio_iodev *iodev  = (struct rtio_iodev *)p2;
    int rc = 0;
    struct rtio_cqe *cqe;
    uint8_t *buf;
    uint32_t buf_len;
    struct rtio_sqe *handle;

    q15_t pkt_temp;
    uint32_t pkt_ts;

    rc = sensor_stream(iodev, &stream_ctx, NULL, &handle);

    if (rc) {
        printk("Failed to start sensor stream %d\n", rc);
        return;
    }

    while(1) {
        while (!device_is_ready(imu)) {
            printk("ICM-42688 device not ready\n");
            k_msleep(100);
        }

        cqe = rtio_cqe_consume_block(&stream_ctx);
        rc = rtio_cqe_get_mempool_buffer(&stream_ctx, cqe, &buf, &buf_len);

        if (rc != 0) {
            printk("get mempool buffer failed %d\n", rc);
            return;
        }

        rtio_cqe_release(&stream_ctx, cqe);

        const struct icm42688_fifo_data* fifo_data = (const struct icm42688_fifo_data*) buf;
        // const struct icm42688_decoder_header *header = &fifo_data->header;
        uint8_t* buf_ptr = buf + sizeof(struct icm42688_fifo_data);
        // struct icm42688_packet* buf_pkt = (struct icm42688_packet*) buf;
        uint16_t fifo_count = fifo_data->fifo_count / sizeof(struct icm42688_packet);
        // int packet_size = sizeof(struct icm42688_packet);

        pkt_temp = buf_ptr[0xd];
        pkt_ts = buf_ptr[0xe];

        // uint8_t* byte_ptr = &(buf_pkt->temp);

        int16_t ax_buf[fifo_count], ay_buf[fifo_count], az_buf[fifo_count];
        int16_t gx_buf[fifo_count], gy_buf[fifo_count], gz_buf[fifo_count];

        // printk("Fifo count: %d\n", fifo_count);

        for (int i = 0; i < fifo_count; i++) {

            
            uint8_t header = buf_ptr[0];
            // int16_t z = (int16_t)sys_le16_to_cpu(buf_pkt->ac_z);
            // q15_t z = (q15_t)buf_pkt->ac_z;
            ax_buf[i] = buf_ptr[0x1];
            ay_buf[i] = buf_ptr[0x3];
            // uint8_t z_accel_h = buf[0x5];
            // uint8_t z_accel_l = buf[0x6];
            // int16_t z_accel = (int16_t)sys_le16_to_cpu((z_accel_h << 8) | z_accel_l);
            // az_buf[i] = buf_ptr[0x5];

            gx_buf[i] = buf_ptr[0x7];
            gy_buf[i] = buf_ptr[0x9];
            gz_buf[i] = buf_ptr[0xb];

            // printk("header: %d\n", header);
            // printk("ax: %d\n", (int)(1000*ldexp((int16_t)sys_le16_to_cpu((buf_ptr[0x1] << 8) | buf_ptr[0x2]), 8-15)));
            // printk("ay: %d\n", (int)(1000*ldexp((int16_t)sys_le16_to_cpu((buf_ptr[0x3] << 8) | buf_ptr[0x4]), 8-15)));
            q15_t zzz = (q15_t)((buf_ptr[0x5] << 8) | buf_ptr[0x6]);
            az_buf[i] = zzz;
            float rawVal = ldexp((float)(zzz*40168), 8-31);
            // float zzf = (float)zzz * (1.0f / 32768.0f);
            // float rawVal = (float)zzz * (1.0f / 1 << 31);
            // printk("az: %d\n", (int)(rawVal*1000));
            // printk("temp: %d\n", buf_ptr[0xd]);

            // break;

            // buf_pkt += 1;
            buf_ptr += 16;


            
        }

        // printk("\n");

        q15_t ax_mean, ay_mean, az_mean;
        q15_t gx_mean, gy_mean, gz_mean;

        arm_mean_q15(ax_buf, fifo_count, &ax_mean);
        arm_mean_q15(ay_buf, fifo_count, &ay_mean);
        arm_mean_q15(az_buf, fifo_count, &az_mean);

        arm_mean_q15(gx_buf, fifo_count, &gx_mean);
        arm_mean_q15(gy_buf, fifo_count, &gy_mean);
        arm_mean_q15(gz_buf, fifo_count, &gz_mean);

        struct sensor_packet s_pkt = {
            .v = 1,
            .flags = 0,
            .time_stamp_ns = pkt_ts,
            .temp = pkt_temp,
            .aX = ax_mean,
            .aY = ay_mean,
            .aZ = az_mean,
            .gX = gx_mean,
            .gY = gy_mean,
            .gZ = gz_mean,
        };

        ble_transmit_cb(&s_pkt);

        rtio_release_buffer(&stream_ctx, buf, buf_len);
    }
}

K_THREAD_DEFINE(imu_producer_tid, 2048, stream_data, (void *)imu, (void *)(&iodev), NULL, K_PRIO_COOP(5), K_INHERIT_PERMS, 0);

int imu_init(void (*transmit_cb)(struct sensor_packet*)) {
    int ret = device_is_ready(imu);
    if (!ret) {
        printk("ICM-42688 device not ready\n");
        return -1;
    }

    struct sensor_value ticks = {.val1 = 512, .val2 = 0};
    ret = sensor_attr_set(imu, SENSOR_CHAN_ALL, SENSOR_ATTR_BATCH_DURATION, &ticks);

    if (ret) {
        printk("Unable to update FIFO size\n");
        return ret;
    }

    ble_transmit_cb = transmit_cb;
    return 0;
}

int imu_start_streaming() {
    

    // k_thread_create(&thread_id, thread_stack, TASK_STACK_SIZE, stream_data, (void *)imu, (void *)(&iodev), NULL, K_PRIO_COOP(5), K_INHERIT_PERMS, K_FOREVER);
    k_thread_start(imu_producer_tid);

    return 0;
}