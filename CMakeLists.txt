cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ono-firmware-v10)



file(GLOB_RECURSE sources src/*.c src/*.h)

#add_executable(app ${sources})

target_include_directories(app PUBLIC src)

add_subdirectory(Fusion/Fusion)
target_link_libraries(app PRIVATE Fusion)
#zephyr_link_libraries(m)

target_sources(app PRIVATE src/main.c src/imu.h src/imu.c src/pmic.h src/pmic.c src/haptics.h src/haptics.c src/pressure.h src/pressure.c src/ble.h src/ble.c)
