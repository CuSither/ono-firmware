#include <imu.h>

#define STREAMDEV_ALIAS DT_ALIAS(imu)
#define STREAMDEV_DEVICE DEVICE_DT_GET(STREAMDEV_ALIAS)
static const struct device *const imu = STREAMDEV_DEVICE;

SENSOR_DT_STREAM_IODEV(iodev, STREAMDEV_ALIAS,
    {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE},
    {SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_NOP});

/* ToDo: Calculate how much memory is actually needed */
RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, 4, 4, 128, 16, 4);

void (*ble_transmit_cb)(struct sensor_packet*);

static FusionAhrs ahrs;
static FusionOffset offset;

static uint16_t prev_timestamp = 0;

#define time_scaling 32.768/32.0

static float zVel = 0;
int counter = 0;

const FusionMatrix accelerometerMisalignment = {-0.92624318, -0.35990648, -0.11198617, -0.31824553, 0.58752912, 0.74399551, -0.20197367, 0.72475986, -0.65873347};
const FusionVector accelerometerSensitivity = {1.00257195, 1.00059062, 0.99902027};
const FusionVector accelerometerOffset = {0.00194573, -0.00187991, 0.00601532};

const FusionMatrix gyroscopeMisalignment = {-0.92624318, -0.35990648, -0.11198617, -0.31824553, 0.58752912, 0.74399551, -0.20197367, 0.72475986, -0.65873347};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

ATOMIC_INIT(callibrating);
float calSumX, calSumY, calSumZ;
int calCount;

/* ToDo: Consider whether it's necessary to include all of these structs */
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

        float xSum = 0;
        float ySum = 0;
        float zSum = 0;

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

            q15_t x = (q15_t)((buf_ptr[0x1] << 8) | buf_ptr[0x2]);
            q15_t y = (q15_t)((buf_ptr[0x3] << 8) | buf_ptr[0x4]);
            q15_t z = (q15_t)((buf_ptr[0x5] << 8) | buf_ptr[0x6]);

            q15_t gx = (q15_t)((buf_ptr[0x7] << 8) | buf_ptr[0x8]);
            q15_t gy = (q15_t)((buf_ptr[0x9] << 8) | buf_ptr[0x10]);
            q15_t gz = (q15_t)((buf_ptr[0xb] << 8) | buf_ptr[0xc]);
            
            float axVal = ldexp((float)(x*16), -15);
            float ayVal = ldexp((float)(y*16), -15);
            float azVal = ldexp((float)(z*16), -15);

            float gxVal = ldexp((float)(gx*1000), -15);
            float gyVal = ldexp((float)(gy*1000), -15);
            float gzVal = ldexp((float)(gz*1000), -15);

            uint16_t ts = ((buf_ptr[0xe] << 8) | buf_ptr[0xf]);
            uint16_t td;

            if (ts < prev_timestamp) {
                td = ts + (UINT16_MAX - prev_timestamp);
            } else {
                td = ts - prev_timestamp;
            }

            prev_timestamp = ts;

            float td_sec = (float)td / 1000000.0f * (float)time_scaling;

            FusionVector gyroscope = {gxVal, gyVal, gzVal};
            FusionVector accelerometer = {axVal, ayVal, azVal};

            gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
            accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

            gyroscope = FusionOffsetUpdate(&offset, gyroscope);

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, td_sec);

            // printk("%.5f, %.5f, %.5f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);

            const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
            // const FusionVector linear = FusionAhrsGetLinearAcceleration(&ahrs);

            // xSum += earth.axis.x;
            // ySum += earth.axis.y;
            // zSum += earth.axis.z;

            if (counter >= 100) {
                zVel += earth.axis.z * 1000.0 * td_sec;
            } else {
                counter++;
            }

            xSum += axVal;
            ySum += ayVal;
            zSum += azVal;

            buf_ptr += 16;
        }

        zVel *= 0.99;

        float xMean = xSum / (float)fifo_count;
        float yMean = ySum / (float)fifo_count;
        float zMean = zSum / (float)fifo_count;


        // printk("%.5f, %.5f, %.5f\n", xMean, yMean, zMean);

        // q15_t ax_mean, ay_mean, az_mean;
        q15_t gx_mean, gy_mean, gz_mean;

        // arm_mean_q15(ax_buf, fifo_count, &ax_mean);
        // arm_mean_q15(ay_buf, fifo_count, &ay_mean);
        // arm_mean_q15(az_buf, fifo_count, &az_mean);

        // arm_mean_q15(gx_buf, fifo_count, &gx_mean);
        // arm_mean_q15(gy_buf, fifo_count, &gy_mean);
        // arm_mean_q15(gz_buf, fifo_count, &gz_mean);


        if (atomic_test_bit(callibrating, 0)) {
            calSumX += xMean;
            calSumY += yMean;
            calSumZ += zMean;
            calCount++;

        } else {
            struct sensor_packet s_pkt = {
                .v = 1,
                .flags = 0,
                .time_stamp_ns = pkt_ts,
                .temp = pkt_temp,
                .aX = xMean,
                .aY = yMean,
                .aZ = zVel,
                .gX = gx_mean,
                .gY = gy_mean,
                .gZ = gz_mean,
            };

            ble_transmit_cb(&s_pkt);
        }

        rtio_release_buffer(&stream_ctx, buf, buf_len);
    }
}

static arm_matrix_instance_f32 least_squares(arm_matrix_instance_f32 A, arm_matrix_instance_f32 b, int numColumns, int numRows) {
    arm_matrix_instance_f32 AT;
    arm_matrix_instance_f32 ATMA;
    arm_matrix_instance_f32 ATMAI;
    arm_matrix_instance_f32 x;

    
}

static void callibrate() {
    int numSamples = 20;
    float32_t SAMPLES_f32[3*numSamples];
    float32_t D_f32[9*numSamples];
    arm_matrix_instance_f32 SAMPLES;
    arm_matrix_instance_f32 D;


    for (int i = 0; i < numSamples; i++) {
        calSumX = 0;
        calSumY = 0;
        calSumZ = 0;
        calCount = 0;

        printk("Taking sample %d\n", i+1);

        for (int j = 5; j > 0; j--) {
            printk("Starting to sample in... %d\n");
            k_msleep(1000);
        }

        /* Is this a robust enough way to prevent race conditions? */
        atomic_set_bit(callibrating, 0);
        m_sleep(5000);
        atomic_clear_bit(callibrating, 0);

        printk("Finished sampling\n");

        float mX = calSumX / (float)calCount;
        float mY = calSumY / (float)calCount;
        float mZ = calSumZ / (float)calCount;

        printk("Sample value is {%.4f, %.4f, %.4f}\n", mX, mY, mZ);
        SAMPLES_f32[i*3] = mX;
        SAMPLES_f32[i*3+1] = mY;
        SAMPLES_f32[i*3+2] = mZ;

        D_f32[i*9] = mX * mX;
        D_f32[i*9+1] = mY * mY;
        D_f32[i*9+2] = mZ * mZ;
        D_f32[i*9+3] = 2.0 * mX * mY;
        D_f32[i*9+4] = 2.0 * mX * mZ;
        D_f32[i*9+5] = 2.0 * mY * mZ;
        D_f32[i*9+6] = 2.0 * mX;
        D_f32[i*9+7] = 2.0 * mY;
        D_f32[i*9+8] = 2.0 * mZ;
    }

    arm_mat_init_f32(&SAMPLES, numSamples, 3, (float32_t *)SAMPLES_f32);
    arm_mat_init_f32(&D, numSamples, 9, (float32_t *)D_f32);


    


}

K_THREAD_DEFINE(imu_producer_tid, 4096, stream_data, (void *)imu, (void *)(&iodev), NULL, K_PRIO_COOP(5), K_INHERIT_PERMS, 0);

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

    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, 1024);

    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.6f,
            .gyroscopeRange = 1000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * 1024, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // k_thread_create(&thread_id, thread_stack, TASK_STACK_SIZE, stream_data, (void *)imu, (void *)(&iodev), NULL, K_PRIO_COOP(5), K_INHERIT_PERMS, K_FOREVER);
    k_thread_start(imu_producer_tid);

    callibrate();


    return 0;
}

int imu_start_streaming() {
    

    return 0;
}