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

#include "Rmt_rx_uart.h"
#include <esp_attr.h>
#include <esp_clk_tree.h> //esp_clk_tree_src_get_freq_hz

// The best (highest) resolution_hz is such that 20 uart bits still fit in 15 bit (32767) duration  
// i.e: t20bits = 20 / baud = 32767 / resolution_hz
// max_resolution_hz <= baud * 32767 / 20 <= baud * 1639 
static uint32_t get_best_resolution_hz(uint32_t baud) {
  uint32_t apb_hz;
  esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, &apb_hz);
  uint64_t max_resolution_hz = (uint64_t)baud * 1639;
  uint32_t div = apb_hz / max_resolution_hz + 1;
  if(div > 128) div = 128; //different chips have different RMT implementations: for example ESP32 has a 8-bit divider, ESP32 a fractinal divider with factors of 1.0-128.0 and 256.
  return apb_hz / div; //don't do rounding: on ESP32S3 for example 80Mhz/3 = 26666666 works, but 26666667 does not work as it set the clock to div2 apparently
}

//receive handler, just pushes the symbols in a buffer for later processing - running in ISR context
IRAM_ATTR static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
  Rmt_rx_uart* ser = ( Rmt_rx_uart*)user_data;

  ser->rx_sym.push((uint8_t*)edata->received_symbols, edata->num_symbols * 4);

  //restart receiver if last symbol was received
  if(edata->flags.is_last) {
    rmt_receive(ser->rx_handle, ser->rx_raw_symbols, sizeof(ser->rx_raw_symbols), &ser->rx_config); 
  }

  return false; //no yield needed
}

void Rmt_rx_uart::begin(uint32_t baud, int8_t rxPin, uint32_t resolution_hz) {
  //NOTE: is there really no way to get real resolution_hz from rmt_rx driver???
  if(resolution_hz == 0) resolution_hz = get_best_resolution_hz(baud);
  rmt_bit_len = (resolution_hz + (baud/2)) / baud;

  rx_data.begin(rx_data_buffer, sizeof(rx_data_buffer));
  rx_sym.begin(rx_sym_buffer, sizeof(rx_sym_buffer));

  rmt_rx_channel_config_t rx_chan_config = {};
  rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;   // select source clock
  rx_chan_config.resolution_hz = resolution_hz;   // tick resolution  
  rx_chan_config.mem_block_symbols = 64;//SOC_RMT_MEM_WORDS_PER_CHANNEL; // memory block size in symbols
  rx_chan_config.intr_priority = 0;               //RMT interrupt priority, if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3) 
  rx_chan_config.gpio_num = (gpio_num_t)rxPin;    // GPIO number
  rx_chan_config.flags.invert_in = false;         // do not invert input signal
  rx_chan_config.flags.with_dma = false;          // do not need DMA backend
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_handle));

  //register call backs
  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = rx_done_callback;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_handle, &cbs, this));

  //Serial.printf("enable rx channel\r\n");
  ESP_ERROR_CHECK(rmt_enable(rx_handle));

  //E (420) rmt: rmt_receive(395): signal_range_min_ns too big
  //signal_range_min_ns depends on chip: ESP32 max 255 clk_apb cycles ~= 3000ns, ESP32S3 max 255 resolution_hz cycles
  uint32_t min_ns = (uint32_t)(1000000000/(baud*10)); // 1/10 of a bit (min UART pulse width is 1 bit)
  if(min_ns>3000) min_ns=3000;
  rx_config = {};
  rx_config.signal_range_min_ns = min_ns;
  rx_config.signal_range_max_ns = (uint32_t)(1000000000/(baud/15)); // set to 15 bits (max UART pulse width is 9 bits)
  //ESP32:  E (10) rmt: rmt_receive(366): partial receive not supported 
  rx_config.flags.en_partial_rx = 1;  // enable partial 

  ESP_ERROR_CHECK(rmt_receive(rx_handle, rx_raw_symbols, sizeof(rx_raw_symbols), &rx_config));
}

int Rmt_rx_uart::available(void) {
  rx_decode_symbols();
  return rx_data.used();
}

int Rmt_rx_uart::peek(uint16_t offset) {
  rx_decode_symbols();
  if(rx_data.used() <= offset) return -1;
  return rx_data.peek(offset);
}

size_t Rmt_rx_uart::read(uint8_t *buffer, size_t size) {
  rx_decode_symbols();
  return rx_data.pop(buffer, size);
}

void Rmt_rx_uart::rx_decode_symbols() {
  int sym_len = rx_sym.used() / 4;
  for(int i=0;i<sym_len;i++) {
    rmt_symbol_word_t sym;
    rx_sym.pop((uint8_t*)&sym, 4);
    rx_decode_symbol(sym);
  }
}

void Rmt_rx_uart::rx_decode_symbol(rmt_symbol_word_t sym) {
  rx_fill_bits(sym.duration0, sym.level0);
  rx_fill_bits(sym.duration1, sym.level1);
}

void Rmt_rx_uart::rx_fill_bits(uint32_t duration, uint32_t level) {
  int rmt_bit_numbers = (duration + (rmt_bit_len / 2)) / rmt_bit_len;

  //Serial.printf("%d%c%d ",(int)duration,(level?'H':'L'),(int)rmt_bit_numbers);

  // all remaining bits are high
  if (rmt_bit_numbers == 0 && level) {
      rmt_bit_numbers = 10 - rx_bit_num;
  }

  int from = rx_bit_num;
  int to = rx_bit_num + rmt_bit_numbers;
  for (int j = from; j < to; ++j) {
    if (rx_bit_num == 0 && level) {
      //Serial.printf("not a start bit "); 
      return;
    }
    rx_bit_num++;
    if (rx_bit_num == 10 && !level) {
      //Serial.printf("not a stop bit ");
      return;
    }
    rx_byte |= (level << j);
    if (rx_bit_num == 10) {
      rx_byte >>= 1;
      uint8_t data = rx_byte & 0x00FF;
      rx_data.push(&data, 1);
      //Serial.printf("_rx_data=%d ", data);
      rx_bit_num = 0;
      rx_byte = 0;
    }
  }
}



