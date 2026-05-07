#ifndef SERVO_ULTRASONIC_H
#define SERVO_ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- UART (if you want to use these from here too) ----
void send_char(char c);
void send_string(const char *s);
void send_uint16(uint16_t v);

// ---- Servo ----
void setup_servo(void);
void set_servo_us(uint16_t us);
extern volatile uint16_t servo_angle_us; 

// ---- Ultrasonic ----
void setup_ultrasonic(void);
uint16_t measure_distance_cm(void);

// Debug + full sweep helper
void debug_print(uint16_t angle_us, uint16_t distance_cm);
uint16_t ultrasonic_scan(void);

#ifdef __cplusplus
}
#endif

#endif
