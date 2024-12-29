/*=============================================================================
MIT License

Copyright (c) 2024 qqqlab https://github.com/qqqlab/ESP32-Serial-RMT-UART

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================*/

#include "driver/rmt_tx.h"
#include "Ringbuffer.h"

class Rmt_tx_uart {
public:
  void begin(uint32_t baud, int8_t txPin, uint32_t resolution_hz = 0);
  size_t write(uint8_t *buffer, size_t size);
  int availableForWrite(void) {
    return tx_data.available();
  }

//NOTE: all vars should be private, but some needed for callbacks

private: 
  uint8_t tx_data_buffer[512+1]; //Ringbuffer uses 1 extra byte
  rmt_channel_handle_t tx_handle = NULL;
  rmt_encoder_handle_t tx_simple_encoder = NULL;
  rmt_transmit_config_t tx_config;
public:
  uint16_t rmt_bit_len;
  Ringbuffer tx_data;
  bool tx_busy; 
};
