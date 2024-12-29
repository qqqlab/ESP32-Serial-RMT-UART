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

#include "ESP32_SerialRMT.h"

void SerialRMT::begin(unsigned long baud, int8_t rxPin, int8_t txPin) {
  uint32_t clk = 0;
  if(rxPin >= 0) {
    rx.begin(baud, rxPin, clk);
  }
  if(txPin >= 0) {
    tx.begin(baud, txPin, clk);
  }
}

int SerialRMT::available(void) {
  return rx.available();
}

int SerialRMT::availableForWrite(void) {
  return tx.availableForWrite();
}

int SerialRMT::peek(void) {
  return rx.peek(0);
}

size_t SerialRMT::read(uint8_t *buffer, size_t size) {
  return rx.read(buffer, size);
}

size_t SerialRMT::write(uint8_t *buffer, size_t size) {
  return tx.write(buffer, size);
}
