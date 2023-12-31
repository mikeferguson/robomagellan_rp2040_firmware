/*
 * Copyright (c) 2013-2023, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ETHERBOTIX_HPP__
#define __ETHERBOTIX_HPP__

#define ETHERBOTIX_ID       253

// These are from dynamixel.hpp
#define DYN_READ_DATA       2
#define DYN_WRITE_DATA      3

// Etherbotix Register table
#define REG_MODEL_NUMBER    0   // 16-bit model number
#define REG_VERSION         2
#define REG_ID              3   // Always 253 (same as arbotix)
#define REG_BAUD_RATE       4   // Applied to serial bus
#define REG_DELAY_TIME      5   // Part of serial bus timeout calculation

#define REG_DIGITAL_IN      6   // Corresponds to A0-D7 as digital input
#define REG_DIGITAL_DIR     7   // Digital A0-D7, low for input, high for output
#define REG_DIGITAL_OUT     8   // Digital A0-D7, sets high/low for output pins
#define REG_A0              10  // Read analog value from A0 (raw 12-bit value)
#define REG_A1              12  // Read analog value from A1
#define REG_A2              14  // Read analog value from A2

#define REG_SYSTEM_TIME     16  // 32-bit unsigned system clock
#define REG_SERVO_CURRENT   20  // 16-bit signed current in mA
#define REG_AUX_CURRENT     22  // 16-bit signed current in mA
#define REG_SYSTEM_VOLTAGE  24  // Voltage in 0.1V increments
#define REG_LED             25  // Turns on error led
#define REG_IMU_FLAGS       28  // Information on IMU (read-only)
#define REG_MOTOR_PERIOD    29  // 8-bit motor cycle period (1-100mS, 0 deactives driver)
#define REG_MOTOR_MAX_STEP  30  // Max amount of change in PID setpoint per motor period

#define REG_MOTOR1_VEL      32  // 16-bit motor velocity (ticks/cycle, read-write)
#define REG_MOTOR2_VEL      34  // 16-bit motor velocity (ticks/cycle, read-write)
#define REG_MOTOR1_POS      36  // 32-bit signed position (ticks, read-only)
#define REG_MOTOR2_POS      40  // 32-bit signed position (ticks, read-only)
#define REG_MOTOR1_CURRENT  44  // 16-bit unsigned (raw 12-bit value from ADC)
#define REG_MOTOR2_CURRENT  46  // 16-bit unsigned (raw 12-bit value from ADC)

#define REG_MOTOR1_KP       48  // 32-bit float kp
#define REG_MOTOR1_KD       52  // 32-bit float kd
#define REG_MOTOR1_KI       56  // 32-bit float ki
#define REG_MOTOR1_WINDUP   60  // 32-bit float windup limit

#define REG_MOTOR2_KP       64  // 32-bit float kp
#define REG_MOTOR2_KD       68  // 32-bit float kd
#define REG_MOTOR2_KI       72  // 32-bit float ki
#define REG_MOTOR2_WINDUP   76  // 32-bit float windup limit

#define REG_ACC_X           80  // All these are int16
#define REG_ACC_Y           82
#define REG_ACC_Z           84
#define REG_GYRO_X          86
#define REG_GYRO_Y          88
#define REG_GYRO_Z          90
#define REG_MAG_X           92
#define REG_MAG_Y           94
#define REG_MAG_Z           96
#define REG_USART3_BAUD     98
#define REG_USART3_CHAR     99
#define REG_TIM9_MODE       100
#define REG_TIM9_COUNT      102
#define REG_TIM12_MODE      104
#define REG_TIM12_COUNT     106
#define REG_SPI2_BAUD       108

#define REG_PACKETS_RECV    120
#define REG_PACKETS_BAD     124

#define DEVICE_USART3_DATA  128
#define DEVICE_SPI2_DATA    129

#define DEVICE_BOOTLOADER   192
#define DEVICE_UNIQUE_ID    193

#define DEVICE_M1_TRACE     194
#define DEVICE_M2_TRACE     195

// Storage of register data
typedef struct
{
  uint16_t model_number;
  uint8_t version;
  uint8_t id;
  uint8_t baud_rate;
  uint8_t delay_time;
  uint8_t digital_in;  // Read only, acts as mask when written
  uint8_t digital_dir;
  uint8_t digital_out;
  uint8_t user_io_use;
  uint16_t a0;
  uint16_t a1;
  uint16_t a2;

  uint32_t system_time;
  int16_t servo_current;
  int16_t aux_current;
  uint8_t system_voltage;
  uint8_t led;
  uint16_t unused_26;
  uint8_t imu_flags;
  uint8_t motor_period;
  uint16_t motor_max_step;

  int16_t motor1_vel;
  int16_t motor2_vel;
  int32_t motor1_pos;
  int32_t motor2_pos;
  int16_t motor1_current;
  int16_t motor2_current;

  float motor1_kp;
  float motor1_kd;
  float motor1_ki;
  float motor1_windup;

  float motor2_kp;
  float motor2_kd;
  float motor2_ki;
  float motor2_windup;

  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;

  uint8_t usart3_baud;
  uint8_t usart3_char;
  uint16_t tim9_mode;
  uint16_t tim9_count;
  uint16_t tim12_mode;
  uint16_t tim12_count;
  uint8_t spi2_baud;
  uint8_t unused_109;
  uint16_t unused_110;

  uint32_t unused_112;
  uint32_t unused_116;
  uint32_t packets_recv;
  uint32_t packets_bad;
} registers_t;

#endif // __ETHERBOTIX_HPP__