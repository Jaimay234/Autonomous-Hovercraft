// Servo + ultrasonic scan – single 180° sweep, find largest free gap

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "servo_ultrasonic.h"
#include "imu2.h"      // for UART: send_string / send_uint16

#define FREE_SPACE_THRESHOLD_CM 40

// ---- Servo timing (HS-422 style) ----
// Timer1 with prescaler 8 → 0.5 µs per tick
// Period = 20 ms → 20000 µs → 40000 ticks
#define SERVO_PERIOD_TICKS 40000      // ICR1
#define SERVO_MIN_US       500        // ~0° 500
#define SERVO_MAX_US       2550       // ~180° 2550
#define SERVO_INCREMENT    80         // step size (µs)

#define SERVO_CENTER_US        1500   // mechanical center
#define SERVO_FORWARD_OFFSET_US   0   // tweak if needed (+/- us)

// Convert microseconds to timer ticks (0.5 µs per tick with prescaler 8)
#define US_TO_TICKS(us)    ((us) * 2)

// ---------- Ultrasonic (HC-SR04) pins ----------
#define US_TRIG_DDR   DDRB
#define US_TRIG_PORT  PORTB
#define US_TRIG_PIN   PB5        // TRIG on PB5

#define US_ECHO_DDR   DDRD
#define US_ECHO_PINR  PIND
#define US_ECHO_PIN   PD3        // ECHO on PD3

// ---------- Globals for scan result ----------
volatile uint16_t max_distance_cm = 0;
volatile uint16_t servo_angle_us  = 0;

// --------------------------------------------------
// Servo on OC1A (PB1 / D9)
// --------------------------------------------------
void setup_servo(void) {
    // PB1 (OC1A) as output
    DDRB |= (1<<PB1);

    // Timer1: Fast PWM, ICR1 as TOP, non-inverting on OC1A, prescaler 8
    TCCR1A = (1<<COM1A1) | (1<<WGM11);
    TCCR1B = (1<<WGM13)  | (1<<WGM12) | (1<<CS11);

    ICR1 = SERVO_PERIOD_TICKS;   // 20 ms period
}

void set_servo_us(uint16_t us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    OCR1A = US_TO_TICKS(us);
}

// --------------------------------------------------
// Ultrasonic (HC-SR04) – polling-based
// --------------------------------------------------
void setup_ultrasonic(void)
{
    // TRIG as output, start low
    US_TRIG_DDR  |=  (1 << US_TRIG_PIN);
    US_TRIG_PORT &= ~(1 << US_TRIG_PIN);

    // ECHO as input, no pull-up
    US_ECHO_DDR  &= ~(1 << US_ECHO_PIN);
}

// Measure distance in cm using HC-SR04
// Returns 0 if no echo / timeout
uint16_t measure_distance_cm(void)
{
    uint16_t timeout;
    uint16_t start, end, ticks;

    // 1) Ensure ECHO is low before starting
    timeout = 60000;
    while ((US_ECHO_PINR & (1 << US_ECHO_PIN)) && timeout) {
        _delay_us(1);
        timeout--;
    }
    if (!timeout) return 0;

    // 2) 10 µs TRIG pulse
    US_TRIG_PORT &= ~(1 << US_TRIG_PIN);
    _delay_us(2);
    US_TRIG_PORT |=  (1 << US_TRIG_PIN);
    _delay_us(10);
    US_TRIG_PORT &= ~(1 << US_TRIG_PIN);

    // 3) Wait for ECHO rising edge
    timeout = 60000;
    while (!(US_ECHO_PINR & (1 << US_ECHO_PIN)) && timeout) {
        _delay_us(1);
        timeout--;
    }
    if (!timeout) return 0;

    // Record start time from Timer1 (0.5 µs per tick)
    start = TCNT1;

    // 4) Wait for ECHO falling edge
    timeout = 60000;
    while ((US_ECHO_PINR & (1 << US_ECHO_PIN)) && timeout) {
        _delay_us(1);
        timeout--;
    }
    if (!timeout) return 0;

    end = TCNT1;

    // 5) Compute tick difference (handle wrap at SERVO_PERIOD_TICKS)
    if (end >= start) {
        ticks = end - start;
    } else {
        // Timer wrapped once between start and end
        ticks = (SERVO_PERIOD_TICKS - start) + end;
    }

    // 6) Convert ticks to time (µs)
    // Timer1 prescaler = 8, F_CPU = 16 MHz → 0.5 µs per tick
    float time_us = ticks * 0.5f;

    // 7) Convert time to distance (cm)
    // HC-SR04: dist_cm ≈ time_us / 58
    uint16_t dist = (uint16_t)(time_us / 58.0f + 0.5f);  // +0.5 for rounding

    // Debug print
    send_string("  Dist(cm)=");
    send_uint16(dist);
    send_string("\r\n");

    return dist;
}


/*
void debug_print(uint16_t angle_us, uint16_t distance_cm)
{
    send_string("Angle(us)=");
    send_uint16(angle_us);

    send_string("  Dist(cm)=");
    send_uint16(distance_cm);

    send_string("  Max(cm)=");
    send_uint16(max_distance_cm);

    send_string("  MaxAngle(us)=");
    send_uint16(servo_angle_us);

    send_string("\r\n");
}
*/
// --------------------------------------------------
// Full 180° sweep helper
// Finds widest sector with distance >= FREE_SPACE_THRESHOLD_CM
// and points servo to center of that sector.
// --------------------------------------------------
uint16_t ultrasonic_scan(void)
{
    uint16_t us = SERVO_MIN_US;

    // Go to leftmost & settle
    set_servo_us(SERVO_MIN_US);
    _delay_ms(500);

    max_distance_cm = 0;
    servo_angle_us  = SERVO_MIN_US;

    // Gap tracking variables
    uint8_t  in_gap          = 0;
    uint16_t gap_start_us    = 0;
    uint16_t gap_end_us      = 0;

    uint16_t best_start_us   = 0;
    uint16_t best_end_us     = 0;
    uint16_t best_width_us   = 0;

    // Sweep from MIN to MAX (left → right) and scan
    for (us = SERVO_MIN_US; us <= SERVO_MAX_US; us += SERVO_INCREMENT) {

        uint16_t d = measure_distance_cm();
        // debug_print(us, d);

        // Threshold-based gap detection
        if (d >= FREE_SPACE_THRESHOLD_CM) {
            if (!in_gap) {
                in_gap       = 1;
                gap_start_us = us;
                gap_end_us   = us;
            } else {
                gap_end_us   = us;
            }
        } else {
            if (in_gap) {
                uint16_t width = gap_end_us - gap_start_us;
                if (width > best_width_us) {
                    best_width_us = width;
                    best_start_us = gap_start_us;
                    best_end_us   = gap_end_us;
                }
                in_gap = 0;
            }
        }

        // Step to next position
        uint16_t next_us = us + SERVO_INCREMENT;
        if (next_us <= SERVO_MAX_US) {
            set_servo_us(next_us);
            _delay_ms(200);
        }
    }

    // Close last gap if still open
    if (in_gap) {
        uint16_t width = gap_end_us - gap_start_us;
        if (width > best_width_us) {
            best_width_us = width;
            best_start_us = gap_start_us;
            best_end_us   = gap_end_us;
        }
    }

    uint16_t target_us;

    if (best_width_us > 0) {
        target_us = (uint16_t)((best_start_us + best_end_us) / 2);
        servo_angle_us  = target_us;
        max_distance_cm = FREE_SPACE_THRESHOLD_CM;
    } else {
        // Fallback: straight ahead
        target_us       = SERVO_CENTER_US;
        servo_angle_us  = target_us;
        max_distance_cm = 0;
    }
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

    int16_t corrected_us = (int16_t)target_us + SERVO_FORWARD_OFFSET_US;
    if (corrected_us < SERVO_MIN_US)  corrected_us = SERVO_MIN_US;
    if (corrected_us > SERVO_MAX_US)  corrected_us = SERVO_MAX_US;

    set_servo_us((uint16_t)corrected_us);

    send_string("Scan done. Max gap >= ");
    send_uint16(FREE_SPACE_THRESHOLD_CM);
    send_string(" cm, center pulse (us) = ");
    send_uint16((uint16_t)corrected_us);
    send_string("\r\n");

    return (uint16_t)corrected_us;
}
