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
#pragma once

#include "Ringbuffer.h"
#include "Rmt_version.h"
#include <stddef.h> //size_t

class Rmt_rx_uart {
public:
  uint8_t idle_timeout_bits = 40; //timeout after this many uart bits, set before calling begin()

  bool begin(uint32_t baud, uint32_t gpio_num, int channel = -1); //channel default -1 assigns next available channel
  size_t read(uint8_t *buffer, size_t size);
  int available(void);
  int peek(uint16_t offset);

  void _rx_end_interrrupt_handler(); //NOTE: should be private, but needed ISR callback
#if QQQ_RMT_VERSION == 2
  void _rx_thres_interrrupt_handler(); //NOTE: should be private, but needed ISR callback
#endif

private:
  void rx_start();
  void rx_decode_symbol_block(uint32_t* sym, size_t size);
  void rx_decode_symbol(uint16_t duration, uint8_t level);
  
  int8_t channel;
  uint8_t rx_data_buffer[512+1]; //plus 1 for RingBuffer
  Ringbuffer rx_data; //RTM_LL channel number (NOTE: this is not always the same as the datasheet channel number)
  uint32_t mem_off; //RMTMEM offset for pingpong
  uint32_t* rmtmem = NULL; //pointer to RTMMEM block for channel
  uint32_t rmtmem_buf[SOC_RMT_MEM_WORDS_PER_CHANNEL/2]; //buffer for half RMTMEM
  uint16_t bit_duration; //in rmt clock ticks
  uint8_t decoder_state = 0; //rx_decoder state
  uint8_t decoder_byte = 0; //rx_decoder current partially received byte
  bool initDone = false;
};
