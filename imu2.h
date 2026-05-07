#ifndef IMU2_H
#define IMU2_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- IMU state exposed to other modules ----
extern float yaw;
extern float meaned_yaw;
extern float distance;
 
void imu_init(void);

// Call repeatedly in loop()
void imu_step(uint16_t base_servo_us);

// ---- UART helpers (used by servo_ultrasonic.c) ----
void send_char(char c);
void send_string(const char *s);
void send_uint16(uint16_t v);

#ifdef __cplusplus
}
#endif

#endif // IMU2_H
