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
#include "Rmt_uart_common.h"
#include "Ringbuffer.h"
#include <hal/rmt_ll.h> //RMT low level
#include <esp_attr.h> //IRAM_ATTR
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals

IRAM_ATTR void Rmt_rx_uart::rx_decode_symbol_block(uint32_t* symbols, size_t size) {
  qqq_rtm_item32_t* sym = (qqq_rtm_item32_t*)symbols;
  for(int i=0;i<size;i++) {
    rx_decode_symbol(sym->duration0, sym->level0);
    if(sym->duration0 == 0) return;
    rx_decode_symbol(sym->duration1, sym->level1);
    if(sym->duration1 == 0) return;    
    sym++;
  }
}

IRAM_ATTR void Rmt_rx_uart::rx_decode_symbol(uint16_t duration, uint8_t level) {
  //get number of bits of level, max number sequential 0's or 1's is 9
  uint8_t number_of_bits; 
  if(duration == 0) {
    number_of_bits = 9;
  } else {
    number_of_bits = (duration + (bit_duration / 2)) / bit_duration; //rounded number of bits
    if(number_of_bits == 9) number_of_bits = 1; //round short pulses to 1 bit
    if(number_of_bits > 9) number_of_bits = 9;
  }

  for (uint8_t i = 0; i < number_of_bits; i++) {
    if (rx_bit_num == 9) {
      //stop bit
      if(!level) {
        //expected high stop bit, but received low bit
        rx_bit_num=0; //reset rx_bit_num, the current byte is probably corrupt
        return;
      }else{
        rx_data.push(&rx_byte, 1);
        rx_bit_num = 0;
        rx_byte = 0;
        return;
      }
    }
    if (rx_bit_num == 0) {
      //start bit
      if(level) {
        //waiting for low start bit, but received high bit -> exit
        return;
      } else {
        rx_bit_num++;
      }
    } else {
      //data bit
      rx_byte >>= 1;
      if(level) rx_byte |= 0x80; //lsb is transmitted first, high=1 low=0
      rx_bit_num++;
    }   
  }
}

IRAM_ATTR void Rmt_rx_uart::rx_start() {
  mem_off = 0;
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for RX channel
  rmt_ll_rx_enable(&RMT, channel, true); //re-enable receiver. NOTE: currently ongoing pulse will be captured as first pulse
}

//RMT-v1 / RMT-v2 device
#if QQQ_RMT_VERSION == 2 
  //decode first half buffer or second half of buffer
  IRAM_ATTR void Rmt_rx_uart::_rx_thres_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);
    mem_off = SOC_RMT_MEM_WORDS_PER_CHANNEL/2 - mem_off; //pingpong between 0 and SOC_RMT_MEM_WORDS_PER_CHANNEL/2    
  }

  //end of reception (idle too long)
  IRAM_ATTR void Rmt_rx_uart::_rx_end_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);

    rx_start(); //restart receiver
  }
#elif QQQ_RMT_VERSION == 1
  #error "TODO pingpong for ESP32/ESP32S2"

  //end of reception (idle too long)
  IRAM_ATTR void Rmt_rx_uart::_rx_end_interrrupt_handler() {
    //TODO
  }
#endif //QQQ_RMT_VERSION




bool Rmt_rx_uart::begin(uint32_t baud, uint32_t gpio_num, int channel) {
  //setup RMT and get next free channel if channel<0
  if(!qqq_rmt_rx_setup(this, &channel)) return false;

  //internal data structures
  this->channel = channel;
  rx_data.begin(rx_data_buffer, sizeof(rx_data_buffer));
  rmtmem = &RMTMEM.channels[channel + QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET].symbols[0].val; //RTM RAM memory pointer
  rx_bit_num = 0; //rx_decoder current bit number
  rx_byte = 0; //rc_decoder current partial received byte

  //calculate best divisor fitting idle_timeout_bits uarts bits in a symbol duration for idle detection, and bit_duration
  if(idle_timeout_bits < 20) idle_timeout_bits = 20;
  uint8_t div = qqq_rmt_calc_best_div(baud, idle_timeout_bits, &bit_duration);

  //setup for UART receiver 
  rmt_ll_set_group_clock_src(&RMT, channel, RMT_CLK_SRC_APB, 1, 1, 0); //set source clk = APB div 1
  rmt_ll_rx_set_channel_clock_div(&RMT, channel, div); //set clock divider
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for TX channel
  rmt_ll_rx_set_mem_blocks(&RMT, channel, 1); //block size = SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 (newer chips ESP32S3/ESP32C6) or 64 (older chips ESP32/ESP32S2), one word is two pulses)
  rmt_ll_rx_set_idle_thres(&RMT, channel, idle_timeout_bits * bit_duration); //Set the time length for RX channel before going into IDLE state
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_set_filter_thres(&RMT, channel, bit_duration / 10); //Set RX channel filter threshold (i.e. the maximum width of one pulse signal that would be treated as a noise)
  rmt_ll_rx_enable_filter(&RMT, channel, true); //Enable filter for RX channel
  #if SOC_RMT_SUPPORT_RX_DEMODULATION
    rmt_ll_rx_enable_carrier_demodulation(&RMT, channel, false); //need to switch off, this option defaults to true
  #endif //SOC_RMT_SUPPORT_RX_DEMODULATION

  //RMT device version specific
  #if QQQ_RMT_VERSION == 2
    //threshold interrupt (generate interrupt at limit and at wrap)
    rmt_ll_rx_set_limit(&RMT, channel, SOC_RMT_MEM_WORDS_PER_CHANNEL/2); //new chips only - Set the amount of RMT symbols that trigger the rx_threshold interrupt
    rmt_ll_rx_enable_wrap(&RMT, channel, true); //new chips only - Wrap RX mode: wrap around to first RAM address
    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_THRES(channel), true);
  #elif QQQ_RMT_VERSION == 1
    #error "TODO pingpong for ESP32/ESP32S2"
    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
  #endif //QQQ_RMT_VERSION

  //GPIO MUX from IDF5.3.1 driver/rmt_rx.c
  //gpio_func_sel(config->gpio_num, PIN_FUNC_GPIO);
  //gpio_input_enable(config->gpio_num);
  //gpio_pullup_en(config->gpio_num);
  //esp_rom_gpio_connect_in_signal(config->gpio_num, rmt_periph_signals.groups[group_id].channels[channel_id + RMT_RX_CHANNEL_OFFSET_IN_GROUP].rx_sig, config->flags.invert_in);

  //pin setup - rmt_periph_signals follows datasheet numbering for ESP32S3 0-3=TX, 4-7=RX - so need offset from RMT_LL channel number
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel + QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET].rx_sig, 0);

  //start rx
  rx_start();

  initDone = true;
  return true;
}

int Rmt_rx_uart::available(void) {
  return rx_data.used();
}

int Rmt_rx_uart::peek(uint16_t offset) {
  if(rx_data.used() <= offset) return -1;
  return rx_data.peek(offset);
}

size_t Rmt_rx_uart::read(uint8_t *buffer, size_t size) {
  return rx_data.pop(buffer, size);
}
