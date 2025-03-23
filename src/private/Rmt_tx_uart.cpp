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

/*
This program uses ESP-IDF5 LL interface to the RMT peripheral to create additional hardware UART TX ports
see https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-guides/hardware-abstraction.html

The ESP32 familily uses two RMT generations. The older version has channels 
that can be used as TX or as RX, the newer generation had specific TX and RX channels

Should work with: 

ESP32    | 4 TX+RX channels (old RMT) | TESTED
ESP32-C3 | 2 TX+RX channels (new RMT) |
ESP32-C5 | 2 TX+RX channels (new RMT) |
ESP32-C6 | 2 TX+RX channels (new RMT) |
ESP32-H2 | 2 TX+RX channels (new RMT) |
ESP32-P4 | 4 TX+RX channels (new RMT) |
ESP32-S2 | 2 TX+RX channels (old RMT) |
ESP32-S3 | 4 TX+RX channels (new RMT) | TESTED

ESP32-C61 and ESP32-H21 have no RMT peripheral
*/

#include "Rmt_tx_uart.h"
#include "Rmt_uart_common.h"
#include "Ringbuffer.h"

#include <hal/rmt_ll.h> //RMT low level
#include <esp_attr.h> //IRAM_ATTR
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals
#include <driver/gpio.h> //gpio_set_direction

//called from ISR and from non-ISR context (!)
IRAM_ATTR void Rmt_tx_uart::_tx_end_interrrupt_handler() {
  if (tx_data.used() == 0) {
    tx_busy = false;
    return;
  }

  int written = 0;
  while (written + 6 <= SOC_RMT_MEM_WORDS_PER_CHANNEL) { //need space for one byte = 10 bits = 5 symbols plus 1 terminating symbol

    //get next byte from buffer
    uint8_t byte;
    if (tx_data.pop(&byte, 1) == 0) {
      break;
    }

    //encode the symbol for a data byte and write the symbol to RMT memory
    rmt_symbol_word_t sym;
    uint16_t b = (byte << 1) | (1 << 9);
    for (int i = 0; i < 9; i += 2) {
      sym.duration0 = bit_duration;
      sym.duration1 = bit_duration;
      sym.level0 = (b >> i) & 0x01;
      sym.level1 = (b >> (i + 1)) & 0x01;
      RMTMEM.channels[channel].symbols[written].val = sym.val;
      written++;
    }
  }
  RMTMEM.channels[channel].symbols[written].val = 0x80008000; //terminating symbol high level

  //start transmitter
  rmt_ll_tx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for TX channel
  rmt_ll_tx_start(&RMT, channel);
}

bool Rmt_tx_uart::begin(uint32_t baud, int gpio_num, int channel) {
  if(gpio_num < 0) return false;

  //setup RMT and get next free channel if channel<0
  if(!qqq_rmt_tx_setup(this, &channel)) return false;

  //internal data structures
  this->channel = channel;
  tx_data.begin(tx_data_buffer, sizeof(tx_data_buffer));

  //calculate best divisor for 1 bit, and bit_duration
  uint8_t div = qqq_rmt_calc_best_div(baud, 1, &bit_duration);

  //setup for UART transmitter
#if SOC_RMT_SUPPORT_TX_ASYNC_STOP
  rmt_ll_tx_stop(&RMT, channel);  //newer chips only
#endif //SOC_RMT_SUPPORT_TX_ASYNC_STOP  
  rmt_ll_set_group_clock_src(&RMT, channel, QQQ_RMT_CLK_SRC, 1, 1, 0); //set source clk = APB div 1
  rmt_ll_tx_set_channel_clock_div(&RMT, channel, div); //set clock divider
  rmt_ll_tx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for TX channel
  rmt_ll_tx_set_mem_blocks(&RMT, channel, 1); //block size = SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 (newer chips ESP32S3/ESP32C6) or 64 (older chips ESP32/ESP32S2), one word is two pulses)
  rmt_ll_tx_enable_wrap(&RMT, channel, false);
  rmt_ll_tx_enable_loop(&RMT, channel, false);
  rmt_ll_tx_fix_idle_level(&RMT, channel, 1, true); //idle level 0=low, 1=high; enable true/false
  //rmt_ll_tx_set_limit(rmt_dev_t *dev, uint32_t channel, uint32_t limit) //not needed
  rmt_ll_tx_enable_carrier_modulation(&RMT, channel, false);
  //rmt_ll_tx_set_carrier_high_low_ticks(rmt_dev_t *dev, uint32_t channel, uint32_t high_ticks, uint32_t low_ticks) //not needed
  //rmt_ll_tx_set_carrier_level(rmt_dev_t *dev, uint32_t channel, uint8_t level) //not needed
#if SOC_RMT_SUPPORT_TX_SYNCHRO
  rmt_ll_tx_clear_sync_group(&RMT);  //newer chips only
#endif // SOC_RMT_SUPPORT_TX_SYNCHRO

  //newer chips only - not needed
  //rmt_ll_tx_stop(&RMT, channel); //newer chips only - Stop transmitting for TX channel
  //rmt_ll_tx_enable_dma(&RMT, channel, false); //newer chips only
  //rmt_ll_tx_set_loop_count(rmt_dev_t *dev, uint32_t channel, uint32_t count) //newer chips only
  //rmt_ll_tx_reset_loop_count(rmt_dev_t *dev, uint32_t channel)  //newer chips only
  //rmt_ll_tx_enable_loop_autostop(rmt_dev_t *dev, uint32_t channel, bool enable) //newer chips only
  //rmt_ll_tx_enable_sync(rmt_dev_t *dev, bool enable) //newer chips only
  //rmt_ll_tx_clear_sync_group(rmt_dev_t *dev)
  //rmt_ll_tx_sync_group_add_channels(rmt_dev_t *dev, uint32_t channel_mask) //newer chips only
  //rmt_ll_tx_sync_group_remove_channels(rmt_dev_t *dev, uint32_t channel_mask) //newer chips only
  //rmt_ll_tx_enable_carrier_always_on(rmt_dev_t *dev, uint32_t channel, bool enable) //not needed

  //GPIO MUX
  //gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO); //works without, not needed (?)
  gpio_set_direction((gpio_num_t)gpio_num, GPIO_MODE_OUTPUT); //works without, not needed (?)
  esp_rom_gpio_connect_out_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel].tx_sig, 0, 0);

  //interrupt
  tx_busy = false;
  rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(channel), true);

  //start transmitter to get correct output level
  RMTMEM.channels[channel].symbols[0].val = 0x80008000; //terminating symbol high level
  rmt_ll_tx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for TX channel
  rmt_ll_tx_start(&RMT, channel);

  //clear ringbuffer
  uint8_t dummy;
  while(tx_data.pop(&dummy,1));

  initDone = true;
  return true;
}

size_t Rmt_tx_uart::write(uint8_t *buffer, size_t size) {
  if(!initDone) return 0;
  size_t written = tx_data.push(buffer, size);
  if(!tx_busy) {
    tx_busy = true;
    _tx_end_interrrupt_handler(); //trigger rmt tx
  }
  return written;
}
