#include <imu.h>
#include <pmic.h>
#include <haptics.h>
#include <pressure.h>
#include <ble.h>
#include <zephyr/kernel.h>

void imu_packet_callback(struct sensor_packet* sensor_data) {
    push_packet_to_queue((void*)sensor_data);
}

int main(void) {
    ble_init();

    pmic_init();
    imu_init(imu_packet_callback);
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
