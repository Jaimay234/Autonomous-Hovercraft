#include "imu2.h"
#include "TWI_290.h"
#include "servo_ultrasonic.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1)
#define TX_BUFFER_SIZE 50

#define I2C_ADDRESS 0x68
#define ACCEL_XOUT  0x3B
#define PWR_MGMT_1  0x6B
#define PWR_MGMT_2  0x6C

#define RAD_TO_DEG              57.2958f 
#define GYRO_SENSITIVITY_250DPS 131.0f
#define LSB_SENSITIVITY_2G      16384.0f 


#define SERVO_MIN_US       500
#define SERVO_MAX_US       2550
#define SERVO_PERIOD_TICKS 40000
#define US_TO_TICKS(us)    ((us) * 2)

static const char CRLF[3] = {13, 10};
static volatile uint8_t *msg;
static volatile uint8_t TX_buffer1[TX_BUFFER_SIZE], TX_buffer2[TX_BUFFER_SIZE];

// ---- Exported IMU state ----
float yaw         = 0.0f;
float meaned_yaw  = 0.0f;
float distance    = 0.0f;

// ---- Internal state ----
static float gyro_z_bias  = 0.0f;
static int   loop_counter = 0;
static float velocity_x   = 0.0f;


static const float dt        = 0.05f;
static const float threshold = 0.1f;

static bool  is_moving  = false;
static float x_bias     = 0.0f;
static float y_bias     = 0.0f;
static float z_bias     = 0.0f;
static float yaw_values[10];

static volatile struct {
    uint8_t TX_finished:1;
    uint8_t sample:1;
    uint8_t mode:1;
    uint8_t stop:1;
    uint8_t T1_ovf0:2;
    uint8_t T1_ovf1:2;
} flags_uart;

// ---------- Internal functions ----------
static void setup_uart(void);
static void setup_IMU(void);
static void calibrate_gyro(void);
static void calibrate_acceleration(void);
static void setup_fan1(void);
static void setup_fan2(void);

static void send_reading(int16_t value, char label[], uint8_t crlf);
static void send_float_internal(float value, const char *label, uint8_t crlf);

static void get_and_display_acceleration(float *accel_data);
static void calculate_roll(float ax, float ay, float az);
static void calculate_yaw_internal(float gyro_z);
static void calculate_pitch(float ax, float ay, float az);
static void get_distance_internal(float acceleration_X);
static void yaw_to_ticks_internal(float yaw_deg, uint16_t base_servo_us);

// ====================== PUBLIC API ======================

void imu_init(void)
{
    setup_uart();
    setup_IMU();
    calibrate_gyro();
    calibrate_acceleration();
    setup_fan1();
    setup_fan2();
    // NOTE: servo Timer1 is initialized in setup_servo() from servo_ultrasonic.c
}

void imu_step(uint16_t base_servo_us)
{
    uint8_t buffer[6];
    float accel_data[3];

    get_and_display_acceleration(accel_data);

    // Read gyro Z
    Read_Reg_N(I2C_ADDRESS, 0x47, 6, (int16_t*)buffer);
    int16_t raw_gyro_z = (buffer[0] << 8) | buffer[1];
    float gyro_z = ((float)(raw_gyro_z - gyro_z_bias) / GYRO_SENSITIVITY_250DPS);

    // Update yaw and servo around base_servo_us
    calculate_yaw_internal(gyro_z);

    // *** changed: use raw yaw instead of meaned_yaw for stronger, quicker correction ***
    yaw_to_ticks_internal(yaw, base_servo_us);

    // Deadzone accel X
    if (fabsf(accel_data[0]) < 0.02f) accel_data[0] = 0.0f;

    // Detect motion
    is_moving = (fabsf(accel_data[0]) > 0.08f) || (fabsf(gyro_z) > 0.6f);

    if (is_moving) {
        get_distance_internal(accel_data[0]);
    } else {
        velocity_x = 0.0f;
    }

    // Periodic debug output & orientation updates
    if (loop_counter >= 20) {
        send_float_internal(accel_data[0], "X Accel Value: ", 0);
        send_float_internal(accel_data[1], " Y Accel Value: ", 0);
        send_float_internal(accel_data[2], " Z Accel Value: ", 1);

        calculate_roll(accel_data[0], accel_data[1], accel_data[2]);
        calculate_pitch(accel_data[0], accel_data[1], accel_data[2]);

        send_float_internal(yaw,      " Yaw : ",      0);
        send_float_internal(distance, " Distance : ", 0);

        loop_counter = 0;
    }

    // Sliding window yaw averaging (still computed if you want to print it)
    if (loop_counter < 10) {
        yaw_values[loop_counter] = yaw;
    } else {
        yaw_values[loop_counter - 10] = yaw;
    }

    if ((loop_counter % 10) == 0) {
        float yaw_sum = 0.0f;
        for (int i = 0; i < 10; i++) {
            yaw_sum += yaw_values[i];
        }
        meaned_yaw = yaw_sum / 10.0f;
    }

    loop_counter++;
    _delay_ms(50);
}

// ====================== UART PUBLIC HELPERS ======================

void send_char(char c)
{
    while (!flags_uart.TX_finished);
    flags_uart.TX_finished = 0;
    UDR0 = (uint8_t)c;
    msg = NULL; // not using buffer-chained send here
}

void send_string(const char *s)
{
    while (!flags_uart.TX_finished);
    msg = (volatile uint8_t*)s;
    UDR0 = *msg;
    flags_uart.TX_finished = 0;
}

void send_uint16(uint16_t v)
{
    while (!flags_uart.TX_finished);
    flags_uart.TX_finished = 0;

    itoa(v, (char*)TX_buffer1, 10);
    msg = TX_buffer1;
    UDR0 = *msg;
}

// ====================== INTERNAL IMPLEMENTATIONS ======================

static void setup_uart(void)
{
    UBRR0H = (uint8_t)((UBRR) >> 8);
    UBRR0L = (uint8_t)UBRR;
    UCSR0B |= (1<<RXCIE0) | (1<<RXEN0);
    UCSR0B |= (1<<TXCIE0) | (1<<TXEN0);
    UCSR0C  = (3<<UCSZ00);
    flags_uart.TX_finished = 1;
}

static void setup_IMU(void)
{
    Write_Reg(I2C_ADDRESS, PWR_MGMT_1, 0x00);
    Write_Reg(I2C_ADDRESS, PWR_MGMT_2, 0x00);
    Write_Reg(I2C_ADDRESS, 0x1B, 0x00);
    Write_Reg(I2C_ADDRESS, 0x1C, 0x00);
}

static void calibrate_gyro(void)
{
    int32_t sum = 0;
    for (int i = 0; i < 100; i++) {
        uint8_t buffer[2];
        Read_Reg_N(I2C_ADDRESS, 0x47, 2, (int16_t*)buffer);
        int16_t raw = (buffer[0] << 8) | buffer[1];
        sum += raw;
        _delay_ms(10);
    }
    gyro_z_bias = (float)sum / 100.0f;
}

static void calibrate_acceleration(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    for (int i = 0; i < 100; i++) {
        uint8_t buffer[6];
        Read_Reg_N(I2C_ADDRESS, ACCEL_XOUT, 6, (int16_t*)buffer);

        int16_t raw_x = (buffer[0] << 8) | buffer[1];
        int16_t raw_y = (buffer[2] << 8) | buffer[3];
        int16_t raw_z = (buffer[4] << 8) | buffer[5];

        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        _delay_ms(10);
    }

    x_bias = (float)sum_x / 100.0f;
    y_bias = (float)sum_y / 100.0f;
    z_bias = (float)sum_z / 100.0f;
}

static void setup_fan1(void)
{
    DDRD |= (1<<PD6); // OC0A
    TCCR0A = (1<<COM0A1) | (1<<WGM00) | (1<<WGM01); // Fast PWM, non-inverting
    TCCR0B = (1<<CS01); // prescaler = 8
}

static void setup_fan2(void)
{
    DDRD |= (1<<PD5); // OC0B
    TCCR0A |= (1<<COM0B1) | (1<<WGM00) | (1<<WGM01); // Fast PWM, non-inverting
    TCCR0B |= (1<<CS01); // prescaler = 8
}

// ---------------- UART TX ISR ----------------

ISR(USART_TX_vect)
{
    if (msg == NULL) {
        flags_uart.TX_finished = 1;
        return;
    }

    msg++;
    if (*msg) {
        UDR0 = *msg;
    } else {
        flags_uart.TX_finished = 1;
    }
}

// ---------------- Internal UART helpers ----------------

static void send_reading(int16_t value, char label[], uint8_t crlf)
{
    while (!flags_uart.TX_finished);
    flags_uart.TX_finished = 0;

    strcpy((char*)TX_buffer1, label);
    itoa(value, (char*)TX_buffer2, 10);
    strcat((char*)TX_buffer1, (char*)TX_buffer2);
    if (crlf) strcat((char*)TX_buffer1, (char*)CRLF);

    msg = TX_buffer1;
    UDR0 = *msg;
}

static void send_float_internal(float value, const char *label, uint8_t crlf)
{
    while (!flags_uart.TX_finished);
    flags_uart.TX_finished = 0;

    strcpy((char*)TX_buffer1, label);
    dtostrf(value, 6, 3, (char*)TX_buffer2);
    strcat((char*)TX_buffer1, (char*)TX_buffer2);
    if (crlf) strcat((char*)TX_buffer1, (char*)CRLF);

    msg = TX_buffer1;
    UDR0 = *msg;
}

// ---------------- IMU / math helpers ----------------

static void get_and_display_acceleration(float *accel_data)
{
    uint8_t buffer[6];
    Read_Reg_N(I2C_ADDRESS, ACCEL_XOUT, 6, (int16_t*)buffer);

    int16_t raw_x = (buffer[0] << 8) | buffer[1];
    int16_t raw_y = (buffer[2] << 8) | buffer[3];
    int16_t raw_z = (buffer[4] << 8) | buffer[5];

    accel_data[0] = (raw_x - x_bias) / LSB_SENSITIVITY_2G;
    accel_data[1] = (raw_y - y_bias) / LSB_SENSITIVITY_2G;
    accel_data[2] = (raw_z - z_bias) / LSB_SENSITIVITY_2G;
}

static void calculate_roll(float ax, float ay, float az)
{
    float accRoll = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
    send_float_internal(accRoll, "Roll : ", 0);
}

static void calculate_yaw_internal(float gyro_z)
{
    if (fabsf(gyro_z) < threshold) {
        gyro_z = 0.0f;
    }

    yaw += gyro_z * dt;

    if (yaw < -180.0f) yaw += 360.0f;
    if (yaw >  180.0f) yaw -= 360.0f;

    // Optional LED behaviour on PB5 like your old code
    if (fabsf(yaw) < 74.0f) {
        PORTB |= (1 << PB5);
    } else {
        PORTB &= ~(1 << PB5);
    }
}

static void calculate_pitch(float ax, float ay, float az)
{
    float accPitch = -atan2f(ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    send_float_internal(accPitch, " Pitch : ", 0);
}

static void get_distance_internal(float acceleration_X)
{
    float acceleration_mps2 = acceleration_X * 9.80665f;
    if (fabsf(acceleration_mps2) < 0.2f) acceleration_mps2 = 0.0f;

    velocity_x += acceleration_mps2 * dt;
    if (fabsf(velocity_x) < 0.001f) velocity_x = 0.0f;

    if (velocity_x != 0.0f) {
        distance += velocity_x * dt * 100.0f;  // cm
    }
}

// ------------ Servo control using servo_ultrasonic ------------

static void yaw_to_ticks_internal(float yaw_deg, uint16_t base_servo_us)
{
    const float SERVO_GAIN_US_PER_DEG = 40.0f;

    int16_t correction_us = (int16_t)(yaw_deg * SERVO_GAIN_US_PER_DEG);
    int16_t pulse_us      = (int16_t)base_servo_us + correction_us;

    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

    OCR1A = US_TO_TICKS(pulse_us);
}
