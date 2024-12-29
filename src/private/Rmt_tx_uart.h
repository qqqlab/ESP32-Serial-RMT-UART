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
#include <stddef.h> //size_t

class Rmt_tx_uart {
public:
  bool begin(uint32_t baud, uint32_t gpio_num, int channel = -1); //channel default -1 assigns next available channel
  size_t write(uint8_t *buffer, size_t size);
  
  //number of bytes that can be written to the transmitter buffer
  int availableForWrite(void) {
    return tx_data.available();
  }

  //number of bytes in the transmitter buffer (waiting to be transmitted)
  int used(void) {
    return tx_data.used();
  }

  void _tx_end_interrrupt_handler(); //NOTE: should be private, but needed ISR callback

private: 
  uint8_t tx_data_buffer[512+1]; //Ringbuffer uses 1 extra byte
  int8_t channel; //RTM_LL channel number (NOTE: this is not always the same as the datasheet channel number)
  uint16_t bit_duration; //in rmt symbol ticks
  Ringbuffer tx_data;
  volatile bool tx_busy = false;
  bool initDone = false;
};
