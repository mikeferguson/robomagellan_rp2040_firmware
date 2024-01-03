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

#include <stdio.h>
#include "vesc.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define VESC_GET_FW_VER             0
#define VESC_GET_STATUS             4
#define VESC_SET_DUTY               5
#define VESC_SET_CURRENT            6
#define VESC_SET_RPM                8
#define VESC_SET_CONF               13
#define VESC_GET_CONF               14
#define VESC_GET_STATUS_SELECTIVE   50

const uint16_t crc_table[] =
{ 
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t crc16(uint8_t * buffer, uint32_t len)
{
  uint32_t i;
  uint16_t checksum = 0;
  for (uint32_t i = 0; i < len; ++i)
  {
    checksum = (checksum << 8) ^ crc_table[((checksum >> 8) ^ *buffer++) & 0xff];
  }
  return checksum;
}

typedef struct
{
  uint8_t start;
  uint16_t length;
  uint8_t payload[1024];
  uint16_t crc;
} PacketVESC;

PacketVESC packet;
uint16_t packet_idx;

#define UART_BUFFER_SIZE  1024
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint16_t uart_head;
uint16_t uart_tail;

// Current VESC driver state
uint8_t vesc_state;
#define VESC_STATE_SEND_CMD   1
#define VESC_STATE_SEND_READ  2
#define VESC_STATE_RECV       3
// Command to send to VESC
int32_t vesc_rpm_command;
// System time when last command was sent
uint32_t vesc_last_sent;
// State variables
int32_t vesc_current_100;
int32_t vesc_rpm_100;
int16_t vesc_voltage_10;
int32_t vesc_dist;

int32_t vesc_get_current_mA()
{
  return vesc_current_100 / 10;
}

int32_t vesc_get_rpm()
{
  return vesc_rpm_100;
}

int32_t vesc_get_dist()
{
  return vesc_dist;
}

void uart_handler_rx()
{
  while (uart_is_readable(uart1))
  {
    uart_buffer[uart_head] = uart_getc(uart1);
    uart_head = (uart_head + 1) % UART_BUFFER_SIZE;
  }
}

void vesc_init()
{
  uart_init(uart1, 115200);
  gpio_set_function(4, GPIO_FUNC_UART);
  gpio_set_function(5, GPIO_FUNC_UART);
  uart_head = uart_tail = 0;
  irq_set_exclusive_handler(UART1_IRQ, uart_handler_rx);
  irq_set_enabled(UART1_IRQ, true);
  uart_set_irq_enables(uart1, true /* rx */, false /* tx */);

  vesc_rpm_command = 0;
  vesc_last_sent = 0;
  vesc_state = VESC_STATE_SEND_CMD;
}

void vesc_set_rpm(int rpm)
{
  vesc_rpm_command = rpm;
}

/*
 * Incrementally decode a VESC packet.
 *
 * Returns:
 *  -1: failed - packet is aborted
 *   0: still processing
 *   1: successfully read packet
 */
int vesc_decode(uint8_t c)
{
  //printf("%i ", (int)c);
  if (packet_idx == 0)
  {
    if (c == 2 || c == 3)
    {
      packet.start = c;
    }
    else
    {
      // Not a valid start - continue
      return 0;
    }
  }
  else if (packet_idx == 1)
  {
    packet.length = c;
  }
  else if (packet.start == 3 && packet_idx == 2)
  {
    // Handle long packets
    packet.length = (packet.length << 8) + c;
  }
  else
  {
    uint16_t payload_idx = packet_idx - packet.start;
    if (payload_idx >= packet.length)
    {
      uint16_t i = payload_idx - packet.length;
      if (i == 0)
      {
        packet.crc = (uint16_t)(c) << 8;
      }
      else if (i == 1)
      {
        packet.crc += (uint16_t)(c);
      }
      else if (c == 3)
      {
        // Check CRC
        uint16_t crc = crc16(packet.payload, packet.length);
        if (crc != packet.crc)
        {
          printf("CRC invalid %i %i %i %i\n", (int)(crc >> 8), (int)(crc & 0xff), (int)(crc), (int)packet.crc);
          return -1;
        }
        
        // Packet is good
        if (packet.payload[0] == VESC_GET_STATUS_SELECTIVE)
        {
          // First 4 bytes are the mask
          // Next 4 bytes are the input current * 100
          vesc_current_100 = ((packet.payload[5] << 24) + (packet.payload[6] << 16) + (packet.payload[7] << 8) + packet.payload[8]);
          // Next 4 bytes are the RPM * 100
          vesc_rpm_100 = ((packet.payload[9] << 24) + (packet.payload[10] << 16) + (packet.payload[11] << 8) + packet.payload[12]);
          // Next 2 bytes are Voltage * 10
          vesc_voltage_10 = ((packet.payload[13] << 8) + packet.payload[14]);
          // Next 4 bytes is distance travled in (3 * poles) electrical rotations
          vesc_dist = (packet.payload[15] << 24) + (packet.payload[16] << 16) + (packet.payload[17] << 8) + packet.payload[18];
        }

        // Reset for next packet
        packet_idx = 0;

        // Valid packet parsed
        return 1;
      }
      else
      {
        printf("Packet invalid\n");
        packet_idx = 0;
        return -1;
      }
    }
    else
    {
      packet.payload[payload_idx] = c;
    }
  }

  ++packet_idx;
}

int vesc_update(uint32_t system_time)
{
  if (vesc_state == VESC_STATE_SEND_CMD)
  {
    // Rate limit
    if ((system_time - vesc_last_sent) < 25)
      return 0;

    // Send command
    uint8_t buffer[10];
    buffer[0] = 2;  // Short packet start byte
    buffer[1] = 5;  // Payload length
    buffer[2] = VESC_SET_RPM;
    buffer[3] = (vesc_rpm_command >> 24) & 0xff;
    buffer[4] = (vesc_rpm_command >> 16) & 0xff;
    buffer[5] = (vesc_rpm_command >> 8) & 0xff;
    buffer[6] = vesc_rpm_command & 0xff;
    uint16_t crc = crc16(&buffer[2], 5);
    buffer[7] = crc >> 8;
    buffer[8] = crc & 0xff;
    buffer[9] = 3;
    uart_write_blocking(uart1, buffer, 10);

    vesc_state = VESC_STATE_SEND_READ;
    vesc_last_sent = system_time;
  }
  else if (vesc_state == VESC_STATE_SEND_READ)
  {
    // Rate limit
    if ((system_time - vesc_last_sent) < 25)
      return 0;

    // Setup to read back
    packet_idx = 0;

    /*
     * bit 3 = return avg input current multiplied by 100
     * bit 7 = return RPM multiplied by 100
     * bit 8 = return Voltage multiplied by 10
     * bit 13 = return electrical distance traveled
     */
    uint32_t selective_mask = (1 << 3) + (1 << 7) + (1 << 8) + (1 << 13);

    // Read back select state
    uint8_t buffer[10];
    buffer[0] = 2;  // Short packet start byte
    buffer[1] = 5;  // Payload length
    buffer[2] = VESC_GET_STATUS_SELECTIVE;
    buffer[3] = (selective_mask >> 16) & 0xff;
    buffer[4] = (selective_mask >> 16) & 0xff;
    buffer[5] = (selective_mask >> 8) & 0xff;
    buffer[6] = selective_mask & 0xff;
    uint16_t crc = crc16(&buffer[2], 5);
    buffer[7] = crc >> 8;
    buffer[8] = crc & 0xff;
    buffer[9] = 3;
    uart_write_blocking(uart1, buffer, 10);

    vesc_state = VESC_STATE_RECV;
    vesc_last_sent = system_time;
  }
  else  // VESC_STATE_RECV
  {
    // Process buffer
    while (uart_head != uart_tail)
    {
      int code = vesc_decode(uart_buffer[uart_tail]);
      uart_tail = (uart_tail + 1) % UART_BUFFER_SIZE;
      if (code != 0)
      {
        // Note that values are updated
        vesc_state = VESC_STATE_SEND_CMD;
        vesc_last_sent = system_time;
        return code;
      }
    }

    // Packet RX timeout
    if ((system_time - vesc_last_sent) > 1000)
    {
      vesc_state = VESC_STATE_SEND_CMD;
      return 0;
    }
  }
}
