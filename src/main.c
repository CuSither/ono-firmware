#include <imu.h>
#include <pmic.h>
#include <haptics.h>
#include <pressure.h>


int main(void) {
    pmic_init();
    imu_start_streaming();
    haptics_init();
    pressure_init();

    while (1) {
        // haptics_start();
        get_pressure();
        print_battery_state();
        k_msleep(10000);
    }

    return 0;
}
