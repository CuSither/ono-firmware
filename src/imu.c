#include <imu.h>

#define STREAMDEV_ALIAS DT_ALIAS(imu)
#define STREAMDEV_DEVICE DEVICE_DT_GET(STREAMDEV_ALIAS)
static const struct device *const imu = STREAMDEV_DEVICE;

SENSOR_DT_STREAM_IODEV(iodev, STREAMDEV_ALIAS,
    {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE},
    {SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_NOP});

/* ToDo: Calculate how much memory is actually needed */
RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, 4, 4, 1024, 16, 4);

struct sensor_chan_spec temp_chan = { SENSOR_CHAN_DIE_TEMP, 0 };
struct sensor_chan_spec accel_chan = { SENSOR_CHAN_ACCEL_XYZ, 0 };
struct sensor_chan_spec gyro_chan = { SENSOR_CHAN_GYRO_XYZ, 0 };

#define TASK_STACK_SIZE           2048ul
K_THREAD_STACK_DEFINE(thread_stack, TASK_STACK_SIZE);
static struct k_thread thread_id;

static void print_stream(void *p1, void *p2, void *p3) {
    const struct device *dev = (const struct device *)p1;
    struct rtio_iodev *iodev  = (struct rtio_iodev *)p2;
    int rc = 0;
    const struct sensor_decoder_api *decoder;
    struct rtio_cqe *cqe;
    uint8_t *buf;
    uint32_t buf_len;
    struct rtio_sqe *handle;

    /* Same here */
    uint8_t temp_buf[512] = { 0 };
    uint8_t accel_buf[512] = { 0 };
    uint8_t gyro_buf[512] = { 0 };

    struct sensor_q31_data *temp_data = (struct sensor_q31_data *)temp_buf;
    struct sensor_three_axis_data *accel_data = (struct sensor_three_axis_data *)accel_buf;
    struct sensor_three_axis_data *gyro_data = (struct sensor_three_axis_data *)gyro_buf;

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

        rc = sensor_get_decoder(dev, &decoder);

        if (rc != 0) {
            printk("sensor_get_decoder failed %d\n", rc);
            return;
        }

        /* Frame iterator values when data comes from a FIFO */
        uint32_t temp_fit = 0, accel_fit = 0, gyro_fit = 0;
        uint16_t temp_count, xl_count, gy_count, frame_count;

        /* There shouldn't be a circumstance where these channels have different frame counts, possibly can be simplified */
        rc = decoder->get_frame_count(buf, temp_chan, &temp_count);
        rc += decoder->get_frame_count(buf, accel_chan, &xl_count);
        rc += decoder->get_frame_count(buf, gyro_chan, &gy_count);

        if (rc != 0) {
            printk("Failed to get frame count %d\n", rc);
            return;
        }

        frame_count = temp_count + xl_count + gy_count;
        // printk("Frame count: %d\n", frame_count);

        int i = 0;

        while (i < frame_count) {
            int8_t c = 0;

            c = decoder->decode(buf, temp_chan, &temp_fit, 16, temp_data);

            // for (int k = 0; k < c; k++) {
            //     int32_t rawVal = ldexp((float)temp_data->readings[k].value, temp_data->shift - 31);

            //     printk("Temp: %d\n", rawVal);
            // }
            i += c;

            c = decoder->decode(buf, accel_chan, &accel_fit, 16, accel_data);

            // for (int k = 0; k < c; k++) {
            // 	printk("XL data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
            // 	       ", %" PRIq(6) ")\n", dev->name,
            // 	       PRIsensor_three_axis_data_arg(*accel_data, k));
            // }
            i += c;
            
            c = decoder->decode(buf, gyro_chan, &gyro_fit, 16, gyro_data);

            // for (int k = 0; k < c; k++) {
            // 	printk("GY data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
            // 	       ", %" PRIq(6) ")\n", dev->name,
            // 	       PRIsensor_three_axis_data_arg(*gyro_data, k));
            // }
            i += c;
        }

        rtio_release_buffer(&stream_ctx, buf, buf_len);
    }
}

int imu_start_streaming() {
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

    k_thread_create(&thread_id, thread_stack, TASK_STACK_SIZE, print_stream, (void *)imu, (void *)(&iodev), NULL, K_PRIO_COOP(5), K_INHERIT_PERMS, K_FOREVER);
    k_thread_start(&thread_id);

    return 0;
}