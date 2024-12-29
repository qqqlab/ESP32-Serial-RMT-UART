#include <Arduino.h>

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

#include "Rmt_tx_uart.h"
#include <esp_attr.h> //IRAM_ATTR
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

static uint8_t dummy_byte = 0; //dummy byte to keep rmt_transmit() happy

static uint8_t test[100];
static int testcnt = 0;

//transmit handler - running in ISR context.
IRAM_ATTR static size_t tx_encoder_callback(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg) {
  Rmt_tx_uart* ser = ( Rmt_tx_uart*)arg;
  
  size_t written = 0;
  rmt_symbol_word_t *sym = symbols;
  while (written + 5 <= symbols_free) { //need space for one byte = 10 bits = 5 symbols

    //get next byte from buffer
    uint8_t byte;
    if(ser->tx_data.pop(&byte, 1) == 0) {
      *done = 1; //Indicate end of the transaction.
      break;
    }

    if(testcnt<sizeof(test)) test[testcnt++]=byte;

    //encode symbols for byte
    uint16_t b = (byte << 1) | (1 << 9);
    for (int i = 0; i < 9; i += 2) {
      sym->duration0 = ser->rmt_bit_len;
      sym->duration1 = ser->rmt_bit_len;
      sym->level0 = (b >> i) & 0x01;
      sym->level1 = (b >> (i + 1)) & 0x01;
      sym++;
      written++;
    }
  }
  
  return written; //number of symbols written
}

//transaction complete callback - running in ISR context - can't use rmt_transmit() here
IRAM_ATTR static bool tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
  Rmt_tx_uart* ser = ( Rmt_tx_uart*)user_ctx;
  ser->tx_busy = false;
  return false; //no yield needed
}

void Rmt_tx_uart::begin(uint32_t baud, int8_t txPin, uint32_t resolution_hz) {
  

  
  //NOTE: is there really no way to get real resolution_hz from rmt_rx driver???
  if(resolution_hz == 0) resolution_hz = get_best_resolution_hz(baud);
  rmt_bit_len = (resolution_hz + (baud/2)) / baud; //rounded bit length in rmt ticks

  //Serial.printf("begin pin=%d res=%d len=%d\n",txPin,resolution_hz,rmt_bit_len);

  tx_data.begin(tx_data_buffer, sizeof(tx_data_buffer));

  rmt_tx_channel_config_t tx_chan_config = {};
  tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;   // select source clock
  tx_chan_config.gpio_num = (gpio_num_t)txPin;    // GPIO number
  tx_chan_config.mem_block_symbols = 128;//SOC_RMT_MEM_WORDS_PER_CHANNEL; // memory block size in symbols
  tx_chan_config.intr_priority = 0; 
  tx_chan_config.resolution_hz = resolution_hz;   // tick resolution
  tx_chan_config.trans_queue_depth = 1;           // set the number of transactions that can pend in the background
  tx_chan_config.flags.invert_out = false;        // do not invert output signal
  tx_chan_config.flags.with_dma = false;          // do not need DMA backend
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_handle));

  rmt_simple_encoder_config_t simple_encoder_cfg = {};
  simple_encoder_cfg.callback = tx_encoder_callback;
  simple_encoder_cfg.min_chunk_size = 5; //5 symbols = 10 bits = 1 byte. // default 64 - Minimum amount of free space, in RMT symbols, the encoder needs in order to guarantee it always returns non-zero
  simple_encoder_cfg.arg = this;          //Opaque user-supplied argument for callback 
  ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &tx_simple_encoder));

  tx_config = {}; //rmt_transmit_config_t 
  tx_config.loop_count = 0;              //sets the number of transmission loops. After the transmitter has finished one round of transmission, it can restart the same transmission again if this value is not set to zero
  tx_config.flags.eot_level = 1;         //sets the output level when the transmitter finishes
  tx_config.flags.queue_nonblocking = 1; //sets whether to wait for a free slot in the transaction queue when it is full. If this value is set to true, then the function will return with an error code ESP_ERR_INVALID_STATE when the queue is full. Otherwise, the function will block until a free slot is available in the queue.

  //register call backs
  rmt_tx_event_callbacks_t cbs = {};
  cbs.on_trans_done = tx_done_callback;
  ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(tx_handle, &cbs, this));

  tx_busy = false;

  ESP_ERROR_CHECK(rmt_enable(tx_handle));
}

size_t Rmt_tx_uart::write(uint8_t *buffer, size_t size) {
  size_t bytes_written = tx_data.push(buffer, size);
  Serial.printf("write size=%d buflen=%d peek=%c testcnt=%d\n", size, tx_data.used(), tx_data.peek(0), testcnt);
  if(!tx_busy) {
    tx_busy = true;
    Serial.printf("rmt_transmit len=%d peek=%c\n", tx_data.used(), tx_data.peek(0));
    rmt_transmit(tx_handle, tx_simple_encoder, &dummy_byte, 1, &tx_config);
  }
  return bytes_written;
}
