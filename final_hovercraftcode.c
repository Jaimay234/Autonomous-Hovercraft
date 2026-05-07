#include "TWI_290.h"
#include "imu2.h"
#include "servo_ultrasonic.h"

#define THRESHOLD_DISTANCE 20
uint16_t current_distance;

void setup() {
    TWI_init(100000UL);
    setup_servo();        // from servo_ultrasonic.c (Timer1 + OCR1A)
    setup_ultrasonic();   // HC-SR04
    imu_init();           // IMU, UART, fans

    sei();

}

void loop() {

    uint16_t base_servo_us;

    current_distance = 0;
    OCR0A = 0; // Fan1 OFF
    base_servo_us = ultrasonic_scan();    // scan phase – servo sweeps, finds best direction
    meaned_yaw = 0;
    yaw = 0;
    OCR0A = 70; // Fan1 full speed  PROP
    OCR0B = 255; // Fan2 full speed LIFT

    _delay_ms(2000);
    current_distance = measure_distance_cm();;

    while (current_distance > 25) {
        current_distance = measure_distance_cm();
        imu_step(base_servo_us);
        _delay_ms(50);                 

    }

}