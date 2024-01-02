/*
 * Copyright (c) 2012-2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "etherbotix.hpp"
#include "socket.h"
#include "wizchip_conf.h"
#include "timer/timer.h"
#include "w5x00_spi.h"
#include "vesc.h"

const uint ACT_LED = PICO_DEFAULT_LED_PIN;
static wiz_NetInfo g_net_info =
{
  .mac = {0x00, 0x00, 0x42, 0x00, 0x00, 0x42}, // MAC address
  .ip = {192, 168, 0, 42},                     // IP address
  .sn = {255, 255, 255, 0},                    // Subnet Mask
  .gw = {192, 168, 0, 1},                      // Gateway
  .dns = {8, 8, 8, 8},                         // DNS server
  .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

void servo_init()
{
  // 125mhz clock / 131 / 20000 = 50hz
  // 20mS per cycle, so resolution is 1uS
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&cfg, 131);
  pwm_config_set_wrap(&cfg, 20000);
  pwm_init(0, &cfg, false /* do not start */);
}

// Sets the servo pulse/angle (in microseconds)
void servo_set_angle(int pulse)
{
  // Output is 1-2ms pulse
  pwm_set_chan_level(0 /* slice */, 0 /* channel */, pulse);
  pwm_set_enabled(0 /* slice */, true);
}

registers_t registers;  // Register data
uint32_t last_packet, last_motor_cmd;

void udp_callback(uint8_t * buffer, uint16_t len, uint8_t * addr, uint16_t port)
{
  uint8_t * data = buffer;

  // Toss any bad packets
  if ((len < 10) ||
      (data[0] != 0xff) || (data[1] != 'B') || (data[2] != 'O') || (data[3] != 'T'))
  {
    ++registers.packets_bad;
    return;
  }

  // Update return data
  last_packet = registers.system_time;

  // For each packet
  size_t i = 4;
  while (i < len)
  {
    if ((data[i] != 0xff) || (data[i + 1] != 0xff))
    {
      // Packet has become corrupted?
      ++registers.packets_bad;
      return;
    }

    uint8_t id = data[i + 2];
    uint8_t len = data[i + 3];
    uint8_t instruction = data[i + 4];

    if (id == ETHERBOTIX_ID)
    {
      // Process packets for self
      if (instruction == DYN_READ_DATA)
      {
        uint8_t read_addr = data[i + 5];
        uint8_t read_len = data[i + 6];

        // Update anything that isn't periodically updated
        ///user_io_update();

        uint8_t packet[256];
        packet[0] = 0xff;
        packet[1] = 'B';
        packet[2] = 'O';
        packet[3] = 'T';
        packet[4] = 0xff;
        packet[5] = 0xff;
        packet[6] = ETHERBOTIX_ID;
        packet[7] = read_len + 2;
        packet[8] = read_addr;  // we transmit address instead of error
        packet[9 + read_len] = ETHERBOTIX_ID + read_len + 2 + read_addr;  // Init checksum

        if (read_addr >= 128)
        {
          if (read_addr == DEVICE_UNIQUE_ID)
          {
            // TODO: does RP2040 have unique ID?
            for (int j = 0; j < read_len; ++j)
            {
              packet[9 + j] = j;
              packet[9 + read_len] += packet[9 + j];
            }
          }
        }
        else
        {
          // Disable interrupts so we don't slice things like encoder values
          //__disable_irq();
          // Copy packet data
          uint8_t * reg_data = (uint8_t *) &registers;
          reg_data += read_addr;
          for (int j = 0; j < read_len; ++j)
          {
            packet[9 + j] = *(reg_data++);
            packet[9 + read_len] += packet[9 + j];
          }
          //__enable_irq();
        }

        packet[9 + read_len] = 255 - packet[9 + read_len];  // Compute checksum
        sendto(0, packet, read_len + 10, addr, port);
      }
      else if (instruction == DYN_WRITE_DATA)
      {
        uint8_t write_addr = data[i + 5];
        if (write_addr >= 128)
        {
          // No devices implemented
        }
        else
        {
          int j = 0;
          while (j < len - 3)
          {
            if (write_addr + j == REG_LED)
            {
              registers.led = data[i + 6 + j];
              if (data[i + 6 + j] > 0)
                gpio_put(ACT_LED, 1);
              else
                gpio_put(ACT_LED, 0);
            }
            else if (write_addr + j == REG_MOTOR2_VEL)
            {
              // Set drive motor velocity
              int16_t v = data[i + 6 + j] + (data[i + 7 + j] << 8);
              vesc_set_rpm(v);
              last_motor_cmd = registers.system_time;
              ++j;  // uses 2 bytes
            }
            else if (write_addr + j == REG_MOTOR1_POS)
            {
              // Set servo position
              int32_t p = data[i + 6 + j] +
                          (data[i + 7 + j] << 8) +
                          (data[i + 8 + j] << 16) +
                          (data[i + 9 + j] << 24);
              if (p >= 1000 && p <= 2000)
              {
                servo_set_angle(p);
                registers.motor1_pos = p;
                last_motor_cmd = registers.system_time;
              }
              j += 3;  // uses 4 bytes
            }
            else
            {
              // INSTRUCTION ERROR on invalid write?
            }
            ++j;
          }
        }
      }
    }

    i += len + 4;
  }

  // Free buffer
  ++registers.packets_recv;
}

void udp_interface_init()
{
  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  network_initialize(g_net_info);
  print_network_information(g_net_info);

  if (socket(0, Sn_MR_UDP, 6707, 0x00) != 0)
  {
    while (true)
    {
      printf("Failed to open socket\n");
      sleep_ms(250);
    }
  }
}

void SysTick_Handler(void)
{
  ++registers.system_time;
}

int main()
{
  stdio_init_all();
  adc_init();
  gpio_set_function(0, GPIO_FUNC_PWM);  // Servo
  servo_init();
  servo_set_angle(1500);  // Center the servo at 1.5 mS
  adc_gpio_init(26);  // Battery voltage
  adc_gpio_init(27);  // Battery current
  vesc_init();

  registers.model_number = 302;  // Arbotix was 300
  registers.version = 7;
  registers.id = ETHERBOTIX_ID;
  registers.baud_rate = 1;  // 1mbps
  registers.digital_dir = 0;  // all in
  registers.digital_out = 0;
  registers.system_time = last_packet = last_motor_cmd = 0;
  registers.motor_period = 10;  // 10mS period = 100hz
  registers.motor_max_step = 10;
  registers.motor1_kp = registers.motor2_kp = 1.0;
  registers.motor1_kd = registers.motor2_kd = 0;
  registers.motor1_ki = registers.motor2_ki = 0.1;
  registers.motor1_windup = registers.motor2_windup = 400;
  registers.usart3_baud = 34;  // 56700
  registers.usart3_char = 255;  // No terminating character
  registers.packets_recv = registers.packets_bad = 0;

  udp_interface_init();
  wizchip_1ms_timer_initialize(SysTick_Handler);

  gpio_init(ACT_LED);
  gpio_set_dir(ACT_LED, GPIO_OUT);
  while (true)
  {
    uint16_t size;
    if ((size = getSn_RX_RSR(0)) > 0)
    {
      uint8_t addr[4];
      uint16_t port;
      uint8_t buffer[2048];
      int32_t len = recvfrom(0, buffer, 2048, addr, &port);
      if (len > 0)
      {
        udp_callback(buffer, len, addr, port);
      }
      else
      {
        printf("Failed to get packet\n");
      }
    }

    // Apply motor timeout
    if (registers.system_time - last_motor_cmd > 500)
    {
      vesc_set_rpm(0);
    }

    // Process VESC updates
    if (vesc_update(registers.system_time) > 0)
    {
      // Copy over values
      registers.motor2_vel = vesc_get_rpm();
      registers.motor2_pos = vesc_get_dist();
    }

    // Get system voltage in 0.1v increment
    //   adc is 12 bit (4096 count) spread over 3.3v
    //   voltage divider is 15k/1k
    adc_select_input(0);
    registers.system_voltage = (adc_read()/4096.0f) * 3.3f * 16 * 10;

    // Get current in mA:
    //   ACS711: vcc/2 = 0A, 55mV/A
    adc_select_input(1);
    registers.servo_current = ((adc_read() - 2048.0f) / 4096.0f) * 3.3f / 0.055f * 1000;

    // Toggle LED
    if (registers.system_time - last_packet < 500)
    {
      if (registers.system_time % 200 == 0)
        gpio_put(ACT_LED, 1);
      else if (registers.system_time % 100 == 0)
        gpio_put(ACT_LED, 0);
    }
    else
    {
      gpio_put(ACT_LED, 1);
    }
  }
}
