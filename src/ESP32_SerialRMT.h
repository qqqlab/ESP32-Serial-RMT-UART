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

#include <Stream.h>
#include <HardwareSerial.h>
#include "private/Rmt_rx_uart.h"
#include "private/Rmt_tx_uart.h"

class SerialRMT : public Stream {
public:
  static uint8_t txCount(); //max number of TX ports
  static uint8_t rxCount(); //max number of RX ports
  
  bool begin(unsigned long baud, int8_t rxPin, int8_t txPin); //set pin to -1 if not used
  int available(void);
  int availableForWrite(void);
  int peek(void);
  size_t read(uint8_t *buffer, size_t size);
  size_t write(uint8_t *buffer, size_t size);
  void flush(bool txOnly);

  int read(void) {
    uint8_t buffer;
    size_t len = read(&buffer, 1);
    if(len==0) return -1;
    return buffer;
  }
  size_t write(uint8_t v) {
    return write(&v, 1);
  }
  void flush() {
    flush(false);
  }

  // Overrides Stream::readBytes() to be faster
  size_t readBytes(uint8_t *buffer, size_t length) {
    return read((uint8_t *)buffer, length);
  }
  size_t readBytes(char *buffer, size_t length) {
    return read((uint8_t *)buffer, length);
  }
  inline size_t write(const char *buffer, size_t size) {
    return write((uint8_t *)buffer, size);
  }
  inline size_t write(const char *s) {
    return write((uint8_t *)s, strlen(s));
  }
  inline size_t write(unsigned long n) {
    return write((uint8_t)n);
  }
  inline size_t write(long n) {
    return write((uint8_t)n);
  }
  inline size_t write(unsigned int n) {
    return write((uint8_t)n);
  }
  inline size_t write(int n) {
    return write((uint8_t)n);
  }

/* TODO - see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
  ~SerialRMT();
  void end(void);
  void updateBaudRate(unsigned long baud);
  bool setMode(SerialMode mode);
  uint32_t baudRate();
  operator bool() const;
  void setDebugOutput(bool);
  void setRxInvert(bool);
  bool setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin = -1, int8_t rtsPin = -1);
  bool setHwFlowCtrlMode(SerialHwFlowCtrl mode = UART_HW_FLOWCTRL_CTS_RTS, uint8_t threshold = 64);
  size_t setRxBufferSize(size_t new_size);
  size_t setTxBufferSize(size_t new_size);
  bool setRxTimeout(uint8_t symbols_timeout);
  bool setRxFIFOFull(uint8_t fifoBytes);
  void eventQueueReset();
*/

private:
  Rmt_rx_uart rx;
  Rmt_tx_uart tx;
};
