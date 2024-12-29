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

#include "driver/rmt_rx.h"
#include "Ringbuffer.h"
#include <esp_clk_tree.h> //esp_clk_tree_src_get_freq_hz

class Rmt_rx_uart {
public:
  void begin(uint32_t baud, int8_t rxPin, uint32_t resolution_hz = 0);
  size_t read(uint8_t *buffer, size_t size);
  int available(void);
  int peek(uint16_t offset);
  
private:
  void rx_decode_symbols();
  void rx_decode_symbol(rmt_symbol_word_t sym);
  void rx_fill_bits(uint32_t duration, uint32_t level);

//NOTE: all vars should be private, but some needed for callbacks

private:
  uint8_t rx_sym_buffer[256*20+1]; //need 20 bytes (10 bits * uint16_t) plus 1 for RingBuffer
  uint8_t rx_data_buffer[256+1]; //plus 1 for RingBuffer
  Ringbuffer rx_data;
  uint8_t rx_bit_num = 0;
  uint16_t rx_byte = 0;

public:
  uint16_t rmt_bit_len;
  rmt_receive_config_t rx_config;
  rmt_symbol_word_t rx_raw_symbols[64];
  rmt_channel_handle_t rx_handle = NULL;
  Ringbuffer rx_sym; 
};
