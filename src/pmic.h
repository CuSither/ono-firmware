#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/drivers/gpio.h>

void print_battery_state();

int pmic_init();