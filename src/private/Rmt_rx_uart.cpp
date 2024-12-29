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
#include "Rmt_version.h"
#include "Rmt_uart_common.h"
#include "Ringbuffer.h"
#include <hal/rmt_ll.h> //RMT low level
#include <esp_attr.h> //IRAM_ATTR
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals

//=============================================================================
//RMT-v1 / RMT-v2 device version specific
//=============================================================================
#if QQQ_RMT_VERSION == 1

  //end of reception (idle too long)
  IRAM_ATTR void Rmt_rx_uart::_rx_end_interrrupt_handler() {
    //TODO SerialRMT receiver RMT-v1 (ESP32/ESP32S2)
  }

  static void rx_begin_version(int channel) {
    //TODO SerialRMT receiver RMT-v1 (ESP32/ESP32S2)
    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
  }

//=============================================================================
#elif QQQ_RMT_VERSION == 2 
  //decode first half buffer or second half of buffer
  IRAM_ATTR void Rmt_rx_uart::_rx_thres_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    //rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    memcpy(rmtmem_buf, &rmtmem[mem_off], 4*SOC_RMT_MEM_WORDS_PER_CHANNEL/2); //copy RMTMEM chunk to buffer in order to give it back to HW as soon as possible
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);

    mem_off = SOC_RMT_MEM_WORDS_PER_CHANNEL/2 - mem_off; //pingpong between 0 and SOC_RMT_MEM_WORDS_PER_CHANNEL/2

    rx_decode_symbol_block(rmtmem_buf, SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
  }

  //end of reception (idle too long)
  IRAM_ATTR void Rmt_rx_uart::_rx_end_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    //rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    memcpy(rmtmem_buf, &rmtmem[mem_off], 4*SOC_RMT_MEM_WORDS_PER_CHANNEL/2); //copy RMTMEM chunk to buffer in order to give it back to HW as soon as possible
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);

    rx_start(); //restart receiver

    rx_decode_symbol_block(rmtmem_buf, SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
  }

  static void rx_begin_version(int channel) {
    //threshold interrupt (generate interrupt at limit and at wrap)
    rmt_ll_rx_set_limit(&RMT, channel, SOC_RMT_MEM_WORDS_PER_CHANNEL/2); //new chips only - Set the amount of RMT symbols that trigger the rx_threshold interrupt
    rmt_ll_rx_enable_wrap(&RMT, channel, true); //new chips only - Wrap RX mode: wrap around to first RAM address
    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_THRES(channel), true);
  }
#endif //QQQ_RMT_VERSION
//=============================================================================

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
  //decoder_state:
  //0   = waiting for high
  //1   = waiting for low start bit
  //2-9 = get bits 0-7
  //10  = expect high stop bit
  
  //get length of pulse in number of uart bits, max pulse is 9 bits
  uint8_t number_of_bits; 
  if(duration == 0) {
    number_of_bits = 9;
  } else {
    number_of_bits = (duration + (bit_duration / 2)) / bit_duration; //rounded number of bits
    if(number_of_bits == 9) number_of_bits = 1; //round short pulses to 1 bit
    if(number_of_bits > 9) number_of_bits = 9;
  }

  for (uint8_t i = 0; i < number_of_bits; i++) {
    switch(decoder_state) {

    case 0: //waiting for high
      if(level) {
        decoder_state++; //set state to waiting for low start bit
        return;
      }else{
        //keep state waiting for high
        return;
      }
      break;
      
    case 1: //waiting for low start bit
      if(!level) {
        decoder_state++; //set state to get bit0
      }else{
        //waiting for low start bit, but received high bit -> exit
        return;
      }
      break;

    case 2: //get bits 0-7
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      //shift bit in, lsb is transmitted first, high=1 low=0
      decoder_byte >>= 1;
      if(level) decoder_byte |= 0x80; 
      decoder_state++; //set state to next bit or stop bit
      break;

    case 10: //expect high stop bit
      if(level) {
        rx_data.push(&decoder_byte, 1); //save byte (no need to zero decoder_byte, all bits will be shifted in)
        decoder_state = 1; //set state to waiting for low (start bit)
        return;
      }else{
        //expected high stop bit, but received low bit
        //current byte is probably corrupt, don't push it
        decoder_state = 0; //set state to waiting for high level
        return;
      }
      break;
    
    default: //should not get here
      decoder_state = 0; //set state to waiting for high level
    }
  }
}

IRAM_ATTR void Rmt_rx_uart::rx_start() {
  mem_off = 0;
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for RX channel
  rmt_ll_rx_enable(&RMT, channel, true); //re-enable receiver. NOTE: currently ongoing pulse will be captured as first pulse
}

bool Rmt_rx_uart::begin(uint32_t baud, uint32_t gpio_num, int channel) {
  //setup RMT and get next free channel if channel<0
  if(!qqq_rmt_rx_setup(this, &channel)) return false;

  //internal data structures
  this->channel = channel;
  rx_data.begin(rx_data_buffer, sizeof(rx_data_buffer));
  rmtmem = &RMTMEM.channels[channel + QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET].symbols[0].val; //RTM RAM memory pointer
  decoder_state = 0; //rx_decoder state is waitng for high level

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

  //RMT device version specific setup
  rx_begin_version(channel);

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
