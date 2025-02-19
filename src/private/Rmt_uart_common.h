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

#include "Rmt_rx_uart.h"
#include "Rmt_tx_uart.h"
#include <hal/rmt_ll.h> //RMT low level

//Target specific defines
#if CONFIG_IDF_TARGET_ESP32C6
  #define QQQ_RMT_CLK_SRC RMT_CLK_SRC_DEFAULT
  #define QQQ_SOC_MOD_CLK SOC_MOD_CLK_PLL_F80M
#else
  //works for ESP32-S3, others not tested
  #define QQQ_RMT_CLK_SRC RMT_CLK_SRC_APB
  #define QQQ_SOC_MOD_CLK SOC_MOD_CLK_APB
#endif

bool qqq_rmt_rx_setup(Rmt_rx_uart* rx_instance, int *channel);
bool qqq_rmt_tx_setup(Rmt_tx_uart* tx_instance, int *channel);
uint8_t qqq_rmt_calc_best_div(uint32_t baud, int bits_to_fit, uint16_t *bit_duration);

//===================================================================
// RMTMEM (NOTE: RMTMEM access was removed from public API in ESP-IDF5)
//===================================================================
typedef struct {
  union {
    struct {
      uint32_t duration0 : 15; /*!< Duration of level0 */
      uint32_t level0 : 1;     /*!< Level of the first part */
      uint32_t duration1 : 15; /*!< Duration of level1 */
      uint32_t level1 : 1;     /*!< Level of the second part */
    };
    uint32_t val; /*!< Equivalent unsigned value for the RMT item */
  };
} qqq_rtm_item32_t;

typedef struct {
  union {
    struct {
        qqq_rtm_item32_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[SOC_RMT_CHANNELS_PER_GROUP];
    uint32_t value[SOC_RMT_CHANNELS_PER_GROUP * SOC_RMT_MEM_WORDS_PER_CHANNEL];
  };
} qqq_rmt_mem_t;

// RMTMEM address is declared in <target>.peripherals.ld
extern qqq_rmt_mem_t RMTMEM;

//dummy global variable to satisfy ll locking, see components/hal/README.md
//comment this out to see places where IDF wants to have locking
extern uint8_t __DECLARE_RCC_ATOMIC_ENV;
