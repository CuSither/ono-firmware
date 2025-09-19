#include <zephyr/kernel.h>
// #include <zephyr/arch/arm64/arch.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <math.h>

int imu_start_streaming();