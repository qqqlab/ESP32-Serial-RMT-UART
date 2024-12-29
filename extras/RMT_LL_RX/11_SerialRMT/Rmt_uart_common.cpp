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

#include "Rmt_uart_common.h"
#include "Rmt_version.h"

#include <hal/rmt_ll.h> //RMT low level
#include <esp_intr_alloc.h> //esp_intr_alloc
#include <esp_attr.h> //IRAM_ATTR
#include <soc/interrupts.h> //ETS_RMT_INTR_SOURCE
#include <esp_clk_tree.h> //esp_clk_tree_src_get_freq_hz

uint8_t __DECLARE_RCC_ATOMIC_ENV;

static bool qqq_rmt_setup_done = false;

static Rmt_tx_uart* tx[QQQ_TX_CHANNELS] = {}; //index is the RMT_LL channel number
static Rmt_rx_uart* rx[QQQ_RX_CHANNELS] = {}; //RMTv2:index is the RMT_LL channel number; RMTv1: index is RMT_LL-QQQ_RX_CHANNEL_INTR_OFFSET

//RMT Interrupt Handler
IRAM_ATTR static void qqq_rmt_isr_handler(void* arg){
  uint32_t status;

  //TX end interrupt 
  status = rmt_ll_get_tx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_TX_CHANNELS; ch++) {
    if (status & (1 << (ch))) {
      if (tx[ch]) tx[ch]->_tx_end_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(ch));
    }
  }

  //RX end interrupt 
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << (ch + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET))) {
      if (rx[ch]) rx[ch]->_rx_end_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(ch + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET));
    }
  }

  //RMT device version
  #if QQQ_RMT_VERSION == 2  
    //RX threshold interrupt 
    status = rmt_ll_get_rx_thres_interrupt_status(&RMT);
    for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
      if (status & (1 << (ch + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET))) {
        if (rx[ch]) rx[ch]->_rx_thres_interrrupt_handler();
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_THRES(ch + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET));
      }
    }
  #endif //QQQ_RMT_VERSION
}

static void qqq_rmt_setup() {
  //RMT peripheral setup
  if(qqq_rmt_setup_done) return;
  rmt_ll_enable_bus_clock(0, true);
  rmt_ll_reset_register(0);
  //rmt_ll_enable_periph_clock(&RMT, true); //unchanged (off) in rmt_rx.c
  //rmt_ll_mem_force_power_on(&RMT);  //unchanged (off) in rmt_rx.c
  rmt_ll_enable_mem_access_nonfifo(&RMT, true);
  rmt_ll_enable_group_clock(&RMT, true);

  //interrupt - can use ESP_INTR_FLAG_LEVEL1(lowest) to ESP_INTR_FLAG_LEVEL3(highest) priority
  esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, qqq_rmt_isr_handler, NULL, NULL);

  qqq_rmt_setup_done = true;
  return;
}

bool qqq_rmt_rx_setup(Rmt_rx_uart* rx_instance, int *channel) {
  //find next available channel
  int ch = *channel;
  if (ch < 0) {
    for (ch=0; ch<QQQ_RX_CHANNELS; ch++) {
      if (!rx[ch]) break;
    }
  }
  if(ch >= QQQ_RX_CHANNELS) return false; //not enough channels
  if(rx[ch]) return false; //channel already occupied
  rx[ch] = rx_instance;
  *channel = ch + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET; //RMT_LL channel number from Rmt_uart channel number

  qqq_rmt_setup();
  return true;
}

bool qqq_rmt_tx_setup(Rmt_tx_uart* tx_instance, int *channel) {
  //find next available channel
  int ch = *channel;
  if (ch < 0) {
    for (ch=0; ch<QQQ_TX_CHANNELS; ch++) {
      if (!tx[ch]) break;
    }
  }
  if(ch >= QQQ_TX_CHANNELS) return false; //not enough channels
  if(tx[ch]) return false; //channel already occupied
  tx[ch] = tx_instance;
  *channel = ch;

  qqq_rmt_setup();
  return true;
}

// The best (lowest) divisor is such that bits_to_fit uart bits still fit in 15 bit (32767) duration  
// i.e: bits_to_fit period [seconds] = bits_to_fit / baud = 32767 * div / clk
// div >= bits_to_fit * clk / baud / 32767;
uint8_t qqq_rmt_calc_best_div(uint32_t baud, int bits_to_fit, uint16_t *bit_duration) {
  uint32_t apb_hz;
  esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, &apb_hz);
  uint32_t div = apb_hz / baud * bits_to_fit / 32767 + 1;
  if(div>255) div = 255; //limit to 8 bits
  *bit_duration = (apb_hz / div + baud/2) / baud;
  return div;
}
