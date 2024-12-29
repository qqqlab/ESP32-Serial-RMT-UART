/*RMT RX with <hal/rmt_ll.h>
2024-12-27 tested with: 
ESP32 and ESP32S3
ESP-IDF Version  v5.3.2-174-g083aad99cf-dirty
Arduino Version  3.1.0

connect TXPIN to RXPIN

test case for pingpong on ESP32S3 - send 75 low pulses of increasing length, capture with threshold and end interrupts

TEST PASSED

TARGET: esp32s3
loopcnt=1 rx_buf len=100
THR:150H 110L 150H 120L 150H 130L 150H 140L 150H 150L 150H 160L 150H 170L 150H 180L 150H 190L 150H 200L 150H 210L 150H 220L 150H 230L 150H 240L 150H 250L 150H 260L 150H 270L 150H 280L 150H 290L 150H 300L 150H 310L 150H 320L 150H 330L 150H 340L 
THR:150H 350L 150H 360L 150H 370L 150H 380L 150H 390L 150H 400L 150H 410L 150H 420L 150H 430L 150H 440L 150H 450L 150H 460L 150H 470L 150H 480L 150H 490L 150H 500L 150H 510L 150H 520L 150H 530L 150H 540L 150H 550L 150H 560L 150H 570L 150H 580L 
THR:150H 590L 150H 600L 150H 610L 150H 620L 150H 630L 150H 640L 150H 650L 150H 660L 150H 670L 150H 680L 150H 690L 150H 700L 150H 710L 150H 720L 150H 730L 150H 740L 150H 750L 150H 760L 150H 770L 150H 780L 150H 790L 150H 800L 150H 810L 150H 820L 
END:150H 830L 150H 840L 


*/


#define TXPIN 16
#define RXPIN 17
#define CHANNEL 0

#if CONFIG_IDF_TARGET_ESP32
  #define CHANNEL_OFFSET 0 //ESP32 = 0
#elif CONFIG_IDF_TARGET_ESP32S3
  #define CHANNEL_OFFSET 4 //ESP32S3 = 4
#else
  #error "unknown CONFIG_IDF_TARGET"
#endif //CONFIG_IDF_TARGET_ESP32

#include <hal/rmt_ll.h> //RMT low level
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals

uint8_t __DECLARE_RCC_ATOMIC_ENV;

#if SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_RX_CANDIDATES_PER_GROUP
  #define QQQ_RX_CHANNELS ((SOC_RMT_RX_CANDIDATES_PER_GROUP)/2)  //older chip (ESP32 etc) with RMT channels that can be used as TX or RX -> use lower half for TX, higher half for RX
  #define QQQ_RX_CHANNEL_OFFSET QQQ_RX_CHANNELS
#else
  #define QQQ_RX_CHANNELS (SOC_RMT_RX_CANDIDATES_PER_GROUP) //newer chip (ESP32S3 etc) with specific TX and RX RMT channels
  #define QQQ_RX_CHANNEL_OFFSET 0
#endif

#if SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_TX_CANDIDATES_PER_GROUP
  #define QQQ_TX_CHANNELS ((SOC_RMT_TX_CANDIDATES_PER_GROUP)/2)  //older chip (ESP32 etc) with RMT channels that can be used as TX or RX -> use lower half for TX, higher half for RX
#else
  #define QQQ_TX_CHANNELS (SOC_RMT_TX_CANDIDATES_PER_GROUP) //newer chip (ESP32S3 etc) with specific TX and RX RMT channels
#endif

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

uint32_t mem_off; //RMTMEM offset for pingpong
uint32_t* rmtmem = NULL; //pointer to RTMMEM block for channel

//for testing
qqq_rtm_item32_t rx_buf[500];
volatile int rx_cnt = 0;
#define SYMTHR 0x12345678 // (1111 | (1111<<16));
#define SYMEND 0x87654321 // (2222 | (2222<<16));


IRAM_ATTR void rx_buf_push(uint32_t val) {
  if(rx_cnt + 1 >= sizeof(rx_buf) / 4) return;
  rx_buf[rx_cnt].val = val;
  rx_cnt++;
}

IRAM_ATTR void rx_decode_symbol_block(uint32_t* sym, size_t size) {
  if(rx_cnt + size >= sizeof(rx_buf) / 4) return;
  memcpy(&rx_buf[rx_cnt], sym, size*4);
  rx_cnt += size;
}

//new / old RMT device
#if SOC_RMT_SUPPORT_RX_PINGPONG 
  //decode first half buffer
  IRAM_ATTR void _newrmt_rx_thres_interrrupt_handler() {
    rx_buf_push(SYMTHR);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);
    mem_off = SOC_RMT_MEM_WORDS_PER_CHANNEL/2 - mem_off; //pingpong between 0 and SOC_RMT_MEM_WORDS_PER_CHANNEL/2
  }

  //decode second half buffer - wrap around or end of receive
  IRAM_ATTR void _newrmt_rx_end_interrrupt_handler() {
    rx_buf_push(SYMEND);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
    rx_decode_symbol_block(&rmtmem[mem_off], SOC_RMT_MEM_WORDS_PER_CHANNEL/2);
    rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);
    //rmt_ll_rx_enable(&RMT, channel, true); //restart reveicer when end of receive
  }
#else
  #error "TODO pingpong for ESP32/ESP32S2"

  //end of receive
  IRAM_ATTR void _oldrmt_rx_end_interrrupt_handler() {
    
  }
#endif //SOC_RMT_SUPPORT_RX_PINGPONG


//RMT Interrupt Handler
IRAM_ATTR static void qqq_rmt_isr_handler(void* arg){
  uint32_t status;

//new / old RMT device
#if SOC_RMT_SUPPORT_RX_PINGPONG 
  //RX end interrupt 
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_end_interrrupt_handler();
      if(ch == channel) _newrmt_rx_end_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(ch));
    }
  }

  //RX threshold interrupt 
  status = rmt_ll_get_rx_thres_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_thres_interrrupt_handler();
      if(ch == channel) _newrmt_rx_thres_interrrupt_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_THRES(ch));
    }
  }
#else
  #error "TODO pingpong for ESP32/ESP32S2"
  //RX end interrupt 
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for (int ch = 0; ch < QQQ_RX_CHANNELS; ch++) {
    if (status & (1 << ch)) {
      //if (rx[ch]) rx[ch]->_rx_end_interrrupt_handler();
      if(ch == channel) _oldrmt__rx_end_interrrupt_handler();
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
    //rmt_ll_rx_set_carrier_high_low_ticks(rmt_dev_t *dev, uint32_t channel, uint32_t high_ticks, uint32_t low_ticks) // is switched off
    //rmt_ll_rx_set_carrier_level(rmt_dev_t *dev, uint32_t channel, uint8_t level) // is switched off
  #endif //SOC_RMT_SUPPORT_RX_DEMODULATION

  //new / old RMT device
  #if SOC_RMT_SUPPORT_RX_PINGPONG
    rmt_ll_rx_set_limit(&RMT, channel, SOC_RMT_MEM_WORDS_PER_CHANNEL/2); //new chips only - Set the amount of RMT symbols that trigger the rx_threshold interrupt
    rmt_ll_rx_enable_wrap(&RMT, channel, true); //new chips only - Wrap RX mode: wrap around to first RAM address

    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_THRES(channel), true);
  #else
    #error "TODO pingpong for ESP32/ESP32S2"
    //enable interrupt
    rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(channel), true);
  #endif //SOC_RMT_SUPPORT_RX_PINGPONG

  //pin setup from rmt_rx.c
  //gpio_func_sel(config->gpio_num, PIN_FUNC_GPIO);
  //gpio_input_enable(config->gpio_num);
  //gpio_pullup_en(config->gpio_num);
  //esp_rom_gpio_connect_in_signal(config->gpio_num, rmt_periph_signals.groups[group_id].channels[channel_id + RMT_RX_CHANNEL_OFFSET_IN_GROUP].rx_sig, config->flags.invert_in);

  //pin setup
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel + CHANNEL_OFFSET].rx_sig, false);

  //RTM RAM memory pointer
  rmtmem = &RMTMEM.channels[channel + CHANNEL_OFFSET].symbols[0].val;
}




void rx_receive() {
  int channel = CHANNEL;

  rx_cnt = 0;

  digitalWrite(TXPIN,LOW);
  delayMicroseconds(200);
  
  mem_off = 0;
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for RX channel
  rmt_ll_rx_enable(&RMT, channel, true);

  delayMicroseconds(200);
}



void rx_print(String msg, void* ptr, int len) {
  Serial.print(msg);
  Serial.printf(" len=%d",len);
  qqq_rtm_item32_t *sym = (qqq_rtm_item32_t*)ptr;
  for(int i=0; i<len; i++) {
    if(sym->val == SYMTHR) {
      Serial.printf("\nTHR:");
    } else if(sym->val == SYMEND) {
      Serial.printf("\nEND:");
    } else {
      if(sym->duration0 == 0) break;
      Serial.printf("%3d%c ",(int)sym->duration0,(sym->level0?'H':'L'));
      if(sym->duration1 == 0) break;
      Serial.printf("%3d%c ",(int)sym->duration1,(sym->level1?'H':'L'));
    }
    sym++;
  }
  Serial.println();
}



void tx_setup() {
  pinMode(TXPIN,OUTPUT);
  digitalWrite(TXPIN,HIGH);  
}

int loopcnt = 0;

void tx_send() {
  uint32_t t = esp_cpu_get_cycle_count();
  tx_pulses(&t, 75, 100, 150);
  delayMicroseconds(100000); //end pulse

  Serial.printf("loopcnt=%d ",loopcnt);
  rx_print("rx_buf", rx_buf, rx_cnt);

}

//send num_pulses tlow / thigh [us] pulses
void tx_pulses(uint32_t *t,int num_pulses, uint32_t tlow, uint32_t thigh) {
  tlow *= 240;
  thigh *= 240;
  for(int i=0;i<num_pulses;i++) {
    digitalWrite(TXPIN,LOW);
    while(esp_cpu_get_cycle_count() - *t < tlow);
    *t += tlow;
    tlow += 240*10; //increase low pulse length
    digitalWrite(TXPIN,HIGH);
    while(esp_cpu_get_cycle_count() - *t < thigh);
    *t += thigh;
  }
}







void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("TARGET: " CONFIG_IDF_TARGET); //lower case chip name without dash e.g. "esp32s3"

  rx_setup();
  tx_setup();
}

void loop() {
  loopcnt++;
  rx_receive();
  tx_send();
  delay(1000);
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