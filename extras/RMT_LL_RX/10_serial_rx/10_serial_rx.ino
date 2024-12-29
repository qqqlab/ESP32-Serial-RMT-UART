/*
2024-12-28
ESP-IDF Version  v5.3.2-174-g083aad99cf-dirty
Arduino Version  3.1.0

connect TXPIN to RXPIN

test serial reception on RMT-v2 - tested on ESP32S3
*/

#define TXPIN 16
#define RXPIN 17
#define CHANNEL 0

#include "Ringbuffer.h"
#include <hal/rmt_ll.h> //RMT low level
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals

uint8_t __DECLARE_RCC_ATOMIC_ENV;

#if SOC_RMT_SUPPORT_RX_PINGPONG && (SOC_RMT_CHANNELS_PER_GROUP == (2 * SOC_RMT_RX_CANDIDATES_PER_GROUP)) && (SOC_RMT_CHANNELS_PER_GROUP == (2 * SOC_RMT_TX_CANDIDATES_PER_GROUP))
  //RMT-v2 (ESP32S3, ESP32C6, etc)
  //RMT with specific TX and RX RMT channels, with PINGPONG
  //in datasheet channels (n) 0-3 are TX and channels (m) 4-7 are RX channels
  //in RMT_LL both TX and RX channels are numbered 0-3, so for RTMMEM RAM (rx_ll_channel + QQQ_RX_CHANNEL_RTMMEM_OFFSET) is needed to access
  #define QQQ_RMT_VERSION 2
  #define QQQ_TX_CHANNELS (SOC_RMT_TX_CANDIDATES_PER_GROUP)
  #define QQQ_RX_CHANNELS (SOC_RMT_RX_CANDIDATES_PER_GROUP) 
  #define QQQ_RX_CHANNEL_RTMMEM_OFFSET QQQ_TX_CHANNELS
#elif !SOC_RMT_SUPPORT_RX_PINGPONG && (SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_RX_CANDIDATES_PER_GROUP) && (SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_TX_CANDIDATES_PER_GROUP)
  #error "TODO gen1 chip (ESP32, ESP32S2, etc)"
  //RMT-v1 (ESP32, ESP32S2, etc)
  //RMT with channels that can be used as TX or RX, without PINGPONG
  //We use lower half for TX, higher half for RX, so QQQ_RX_RTMMEM_OFFSET = 0
  #define QQQ_RMT_VERSION 1  
  #define QQQ_TX_CHANNELS ((SOC_RMT_TX_CANDIDATES_PER_GROUP)/2)
  #define QQQ_RX_CHANNELS ((SOC_RMT_RX_CANDIDATES_PER_GROUP)/2)  
  #define QQQ_RX_CHANNEL_RTMMEM_OFFSET 0
#else
  #error "Unknown RMT version"
#endif

int rxbaud = 10000;

int channel = CHANNEL;
int gpio_num = RXPIN;

typedef struct {
  union {
    struct {
      uint32_t duration0 : 15; // Duration of level0 
      uint32_t level0 : 1;     // Level of the first part 
      uint32_t duration1 : 15; // Duration of level1
      uint32_t level1 : 1;     // Level of the second part
    };
    uint32_t val; // Equivalent unsigned value for the RMT item
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

uint8_t rx_data_buffer[512+1]; //plus 1 for RingBuffer
Ringbuffer rx_data;
uint32_t mem_off; //RMTMEM offset for pingpong
uint32_t* rmtmem = NULL; //pointer to RTMMEM block for channel
uint32_t bit_duration = 1000000/rxbaud; //in rmt clock ticks
uint8_t rx_bit_num = 0; //rx_decoder current bit number
uint8_t rx_byte = 0; //rx_decoder current partially received byte




IRAM_ATTR void rx_decode_symbol_block(uint32_t* symbols, size_t size) {
  qqq_rtm_item32_t* sym = (qqq_rtm_item32_t*)symbols;
  for(int i=0;i<size;i++) {
    rx_decode_symbol(sym->duration0, sym->level0);
    if(sym->duration0 == 0) return;
    rx_decode_symbol(sym->duration1, sym->level1);
    if(sym->duration1 == 0) return;    
    sym++;
  }
}

IRAM_ATTR void rx_decode_symbol(uint16_t duration, uint8_t level) {
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

IRAM_ATTR void rx_start() {
  mem_off = 0;
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for RX channel
  rmt_ll_rx_enable(&RMT, channel, true); //re-enable receiver. NOTE: currently ongoing pulse will be captured as first pulse
}

//RMT-v1 / RMT-v2 device
#if QQQ_RMT_VERSION == 2 
  //decode first half buffer or second half of buffer
  IRAM_ATTR void _v2_rx_thres_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);
    mem_off = SOC_RMT_MEM_WORDS_PER_CHANNEL/2 - mem_off; //pingpong between 0 and SOC_RMT_MEM_WORDS_PER_CHANNEL/2    
  }

  //end of reception (idle too long)
  IRAM_ATTR void _v2_rx_end_interrrupt_handler() {
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);

    rx_start(); //restart receiver
  }
#elif QQQ_RMT_VERSION == 1
  #error "TODO pingpong for ESP32/ESP32S2"

  //end of reception (idle too long)
  IRAM_ATTR void _v1_rx_end_interrrupt_handler() {
    
  }
#endif //QQQ_RMT_VERSION


//RMT Interrupt Handler
IRAM_ATTR static void qqq_rmt_isr_handler(void* arg){
  uint32_t status;

//RMT device version
#if QQQ_RMT_VERSION == 2  
  //RX end interrupt 
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_end_interrrupt_handler();
      if(ch == channel) _v2_rx_end_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(ch));
    }
  }

  //RX threshold interrupt 
  status = rmt_ll_get_rx_thres_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_thres_interrrupt_handler();
      if(ch == channel) _v2_rx_thres_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_THRES(ch));
    }
  }
#elif QQQ_RMT_VERSION == 1
  #error "TODO pingpong for ESP32/ESP32S2"
  //RX end interrupt 
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_end_interrrupt_handler();
      if(ch == channel) _v1_rx_end_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(ch));
    }
  }
#endif //SOC_RMT_SUPPORT_RX_PINGPONG
}




void rx_setup() {
  //RMT peripheral setup
  rmt_ll_enable_bus_clock(0, true);
  rmt_ll_reset_register(0);
  //rmt_ll_enable_periph_clock(&RMT, true);
  //rmt_ll_mem_force_power_on(&RMT);
  rmt_ll_enable_mem_access_nonfifo(&RMT, true);
  rmt_ll_enable_group_clock(&RMT, true);

  //allocate interrupt
  esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, qqq_rmt_isr_handler, NULL, NULL);




  //internal data structures
  rx_data.begin(rx_data_buffer, sizeof(rx_data_buffer));
  rmtmem = &RMTMEM.channels[channel + QQQ_RX_CHANNEL_RTMMEM_OFFSET].symbols[0].val; //RTM RAM memory pointer

  //setup for UART receiver (rmt_ll_rx channels start with 0, even if datasheet rx chm channels start at 4)
  //sys_conf
  rmt_ll_set_group_clock_src(&RMT, channel, RMT_CLK_SRC_APB, 1, 1, 0); //RMT_SCLK_DIV_NUM + 1 + RMT_SCLK_DIV_A / RMT_SCLK_DIV_B
  //conf0
  rmt_ll_rx_set_channel_clock_div(&RMT, channel, 80); //set clock divider
  rmt_ll_rx_set_mem_blocks(&RMT, channel, 1); //block size = SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 (newer chips ESP32S3/ESP32C6) or 64 (older chips ESP32/ESP32S2), one word is two pulses)
  rmt_ll_rx_set_idle_thres(&RMT, channel, 32767); //0-32767 = Set the time length for RX channel before going into IDLE state

  //conf1
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_enable_filter(&RMT, channel, false); //Enable filter for RX channel
  //TODO rmt_ll_rx_set_filter_thres(&RMT, channel, uint32_t thres) //Set RX channel filter threshold (i.e. the maximum width of one pulse signal that would be treated as a noise)
  #if SOC_RMT_SUPPORT_RX_DEMODULATION
    rmt_ll_rx_enable_carrier_demodulation(&RMT, channel, false); //need to switch off, this option defaults to true
    //rmt_ll_rx_set_carrier_high_low_ticks(rmt_dev_t *dev, uint32_t channel, uint32_t high_ticks, uint32_t low_ticks) //not needed, carrier is switched off
    //rmt_ll_rx_set_carrier_level(rmt_dev_t *dev, uint32_t channel, uint8_t level)//not needed, carrier is switched off
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

  //pin setup from rmt_rx.c
  //gpio_func_sel(config->gpio_num, PIN_FUNC_GPIO);
  //gpio_input_enable(config->gpio_num);
  //gpio_pullup_en(config->gpio_num);
  //esp_rom_gpio_connect_in_signal(config->gpio_num, rmt_periph_signals.groups[group_id].channels[channel_id + RMT_RX_CHANNEL_OFFSET_IN_GROUP].rx_sig, config->flags.invert_in);

  //pin setup
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel + QQQ_RX_CHANNEL_RTMMEM_OFFSET].rx_sig, false);

  rx_start();
}



void rx_print() {
  Serial.printf("len=%d : ",rx_data.used());
  uint8_t c;
  while(rx_data.pop((uint8_t*)&c,1)) {
    //Serial.printf("0x%02X'%c' ", c, c);
    //Serial.printf("%X ", (int)c);
    Serial.printf("%c", c);
  }
  Serial.println();
}



void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("TARGET: " CONFIG_IDF_TARGET); //lower case chip name without dash e.g. "esp32s3"

  Serial1.begin(rxbaud, SERIAL_8N1, -1, TXPIN);

  rx_setup();
}

int loopcnt = 0;

void loop() {
  loopcnt++;
  char buf[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  Serial1.write((uint8_t*)buf, strlen(buf));
  delay(1000);
  Serial.printf("loopcnt=%d ",loopcnt);
  rx_print();
}










//########################################################################################################
#if 0 
//FROM rmt_private.h
#include <stdatomic.h> //atomic_store

typedef enum {
    RMT_FSM_INIT_WAIT,
    RMT_FSM_INIT,
    RMT_FSM_ENABLE_WAIT,
    RMT_FSM_ENABLE,
    RMT_FSM_RUN_WAIT,
    RMT_FSM_RUN,
} rmt_fsm_t;

//FROM rmt_rx.c
esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config, rmt_channel_handle_t *ret_chan)
  // register the channel to group
  rmt_rx_register_to_group(rx_channel, config)
  // reset channel, make sure the RX engine is not working, and events are cleared
  rmt_hal_rx_channel_reset(&group->hal, channel_id);
  // RMT interrupt is mandatory if the channel doesn't use DMA
  esp_intr_alloc_intrstatus(rmt_periph_signals.groups[group_id].irq, isr_flags, (uint32_t)rmt_ll_get_interrupt_status_reg(hal->regs), RMT_LL_EVENT_RX_MASK(channel_id), rmt_rx_default_isr, rx_channel, &rx_channel->base.intr);
  // select the clock source
  rmt_select_periph_clock(&rx_channel->base, config->clk_src)
  // set channel clock resolution
  rmt_ll_rx_set_channel_clock_div(hal->regs, channel_id, real_div);
  // On esp32 and esp32s2, the counting clock used by the RX filter always comes from APB clock
  // no matter what the clock source is used by the RMT channel as the "core" clock
  #if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &rx_channel->filter_clock_resolution_hz);
  #endif
  //config
  rmt_ll_rx_set_mem_blocks(hal->regs, channel_id, rx_channel->base.mem_block_num);
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_HW);
  #if SOC_RMT_SUPPORT_RX_PINGPONG
    rmt_ll_rx_set_limit(hal->regs, channel_id, rx_channel->ping_pong_symbols);
    // always enable rx wrap, both DMA mode and ping-pong mode rely this feature
    rmt_ll_rx_enable_wrap(hal->regs, channel_id, true);
  #endif
  #if SOC_RMT_SUPPORT_RX_DEMODULATION
    // disable carrier demodulation by default, can re-enable by `rmt_apply_carrier()`
    rmt_ll_rx_enable_carrier_demodulation(hal->regs, channel_id, false);
  #endif
  // GPIO Matrix/MUX configuration
  gpio_func_sel(config->gpio_num, PIN_FUNC_GPIO);
  gpio_input_enable(config->gpio_num);
  gpio_pullup_en(config->gpio_num);
  esp_rom_gpio_connect_in_signal(config->gpio_num, rmt_periph_signals.groups[group_id].channels[channel_id + RMT_RX_CHANNEL_OFFSET_IN_GROUP].rx_sig, config->flags.invert_in);
  // initialize other members of rx channel
  atomic_init(&rx_channel->base.fsm, RMT_FSM_INIT);


esp_err_t rmt_receive(rmt_channel_handle_t channel, void *buffer, size_t buffer_size, const rmt_receive_config_t *config)
  // check if we're in a proper state to start the receiver
  rmt_fsm_t expected_fsm = RMT_FSM_ENABLE;
  ESP_RETURN_ON_FALSE_ISR(atomic_compare_exchange_strong(&channel->fsm, &expected_fsm, RMT_FSM_RUN_WAIT), ESP_ERR_INVALID_STATE, TAG, "channel not in enable state");
  // reset memory writer offset
  rx_chan->mem_off = 0;
  rmt_ll_rx_reset_pointer(hal->regs, channel_id);
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_HW);
  // set sampling parameters of incoming signals
  rmt_ll_rx_set_filter_thres(hal->regs, channel_id, filter_reg_value);
  rmt_ll_rx_enable_filter(hal->regs, channel_id, config->signal_range_min_ns != 0);
  rmt_ll_rx_set_idle_thres(hal->regs, channel_id, idle_reg_value);
  // turn on RMT RX machine
  rmt_ll_rx_enable(hal->regs, channel_id, true);
  // saying we're in running state, this state will last until the receiving is done
  // i.e., we will switch back to the enable state in the receive done interrupt handler
  atomic_store(&channel->fsm, RMT_FSM_RUN);

static esp_err_t rmt_rx_enable(rmt_channel_handle_t channel)
  // can only enable the channel when it's in "init" state
  rmt_fsm_t expected_fsm = RMT_FSM_INIT;
  ESP_RETURN_ON_FALSE(atomic_compare_exchange_strong(&channel->fsm, &expected_fsm, RMT_FSM_ENABLE_WAIT), ESP_ERR_INVALID_STATE, TAG, "channel not in init state");
  // --enable interrupt
  rmt_ll_enable_interrupt(hal->regs, RMT_LL_EVENT_RX_MASK(channel_id), true);
  // --set fsm
  atomic_store(&channel->fsm, RMT_FSM_ENABLE); 

static bool IRAM_ATTR rmt_isr_handle_rx_done(rmt_rx_channel_t *rx_chan)
  // --clear intr status
  rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_RX_DONE(channel_id));
  // disable the RX engine, it will be enabled again when next time user calls `rmt_receive()`
  rmt_ll_rx_enable(hal->regs, channel_id, false);
  // --get mem offset
  uint32_t offset = rmt_ll_rx_get_memory_writer_offset(hal->regs, channel_id);
  // notify the user to process the received symbols if the buffer is going to be full
  .received_symbols = trans_desc->buffer,
  .num_symbols = trans_desc->received_symbol_num,
  .flags.is_last = false
  // copy the symbols to the user buffer
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_SW);
  memcpy((uint8_t *)trans_desc->buffer + trans_desc->copy_dest_off, channel->hw_mem_base + rx_chan->mem_off, copy_size);
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_HW);
  // for chips doesn't support ping-pong RX, we should check whether the receiver has encountered with a long frame,whose length is longer than the channel capacity
  #if !SOC_RMT_SUPPORT_RX_PINGPONG
    if (rmt_ll_rx_get_interrupt_status_raw(hal->regs, channel_id) & RMT_LL_EVENT_RX_ERROR(channel_id)) {
      portENTER_CRITICAL_ISR(&channel->spinlock);
      rmt_ll_rx_reset_pointer(hal->regs, channel_id);
      portEXIT_CRITICAL_ISR(&channel->spinlock);
      // this clear operation can only take effect after we copy out the received data and reset the pointer
      rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_RX_ERROR(channel_id));
      ESP_DRAM_LOGE(TAG, "hw buffer too small, received symbols truncated");
    }
  #endif // !SOC_RMT_SUPPORT_RX_PINGPONG
  // switch back to the enable state, then user can call `rmt_receive` to start a new receive
  atomic_store(&channel->fsm, RMT_FSM_ENABLE);
  // notify the user that all RMT symbols are received done
  .received_symbols = trans_desc->buffer,
  .num_symbols = trans_desc->received_symbol_num,
  .flags.is_last = true,



static bool IRAM_ATTR rmt_isr_handle_rx_threshold(rmt_rx_channel_t *rx_chan)
  // --clear intr status
  rmt_ll_clear_interrupt_status(hal->regs, RMT_LL_EVENT_RX_THRES(channel_id));
  // notify the user to process the received symbols if the buffer is going to be full
  .received_symbols = trans_desc->buffer,
  .num_symbols = trans_desc->received_symbol_num,
  .flags.is_last = false,
  // copy the symbols to the user buffer
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_SW);
  memcpy((uint8_t *)trans_desc->buffer + trans_desc->copy_dest_off, channel->hw_mem_base + rx_chan->mem_off, copy_size);
  rmt_ll_rx_set_mem_owner(hal->regs, channel_id, RMT_LL_MEM_OWNER_HW);
  // update the hw memory offset, where stores the next RMT symbols to copy
  rx_chan->mem_off = rx_chan->ping_pong_symbols - rx_chan->mem_off;
    @@ ping_pong_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL / 2
    @@ mem_off = 0 --> ping_pong_symbols --> 0 --> ping_pong_symbols ...

static void IRAM_ATTR rmt_rx_default_isr(void *args)
  uint32_t status = rmt_ll_rx_get_interrupt_status(hal->regs, channel_id);

  #if SOC_RMT_SUPPORT_RX_PINGPONG
    // RX threshold interrupt
    if (status & RMT_LL_EVENT_RX_THRES(channel_id)) {
        if (rmt_isr_handle_rx_threshold(rx_chan)) {
            need_yield = true;
        }
    }
  #endif // SOC_RMT_SUPPORT_RX_PINGPONG

  // RX end interrupt
  if (status & RMT_LL_EVENT_RX_DONE(channel_id)) {
      if (rmt_isr_handle_rx_done(rx_chan)) {
          need_yield = true;
      }
  }


#endif //0
//########################################################################################################