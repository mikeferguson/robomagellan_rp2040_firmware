/*
 * Copyright (c) 2023-2024, Michael E. Ferguson
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

#ifndef __VESC_H__
#define __VESC_H__

// Initialize the VESC driver
void vesc_init();

// Set the command electrical RPM
void vesc_set_rpm(int rpm);

// Set the command using ticks per motor period
void vesc_set_ticks_per_motor_period(int ticks, uint32_t motor_period);

// Get the input/bus current, in milliamps
int32_t vesc_get_current_mA();

// Get the present electrical RPM
int32_t vesc_get_rpm();

// Get the motor ticks per period
int32_t vesc_get_ticks_per_motor_period(uint32_t motor_period);

// Get the distance traveled
// Multiply this by pi/3 to get electrical distance
int32_t vesc_get_dist();

// Periodic update - handles actual serial comms
int vesc_update(uint32_t system_time);

#endif  // __VESC_H__
