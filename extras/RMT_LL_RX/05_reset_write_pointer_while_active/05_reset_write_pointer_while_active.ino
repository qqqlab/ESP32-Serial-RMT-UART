/*RMT RX with <hal/rmt_ll.h>
2024-12-27 tested with: 
ESP32 and ESP32S3
ESP-IDF Version  v5.3.2-174-g083aad99cf-dirty
Arduino Version  3.1.0

connect TXPIN to RXPIN

test case for ESP32 - copy RMTMEM, reset write pointer while receiving, receive rest of pulses - PASSED
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
#endif

#include <hal/rmt_ll.h> //RMT low level
#include <esp_rom_gpio.h> //esp_rom_gpio_connect_out_signal
#include <soc/rmt_periph.h> //rmt_periph_signals

uint8_t __DECLARE_RCC_ATOMIC_ENV;

int loopcnt = 0;
int channel = CHANNEL;
int gpio_num = RXPIN;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("TARGET: " CONFIG_IDF_TARGET); //lower case chip name without dash e.g. "esp32s3"

  rx_setup();
  tx_setup();
}

void loop() {
  loopcnt++;
  rx_clear(loopcnt);
  rx_receive();
  tx_send();

  rx_print();
  print_rmt();
  delay(1000);
}
void rx_setup() {
  int gpio_num = RXPIN;
  int channel = CHANNEL;

  //RMT peripheral setup
  rmt_ll_enable_bus_clock(0, true);
  rmt_ll_reset_register(0);
  //rmt_ll_enable_periph_clock(&RMT, true);
  //rmt_ll_mem_force_power_on(&RMT);
  rmt_ll_enable_mem_access_nonfifo(&RMT, true);
  rmt_ll_enable_group_clock(&RMT, true);

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
#if SOC_RMT_SUPPORT_RX_DEMODULATION
  rmt_ll_rx_enable_carrier_demodulation(&RMT, channel, false);
#endif

  //pin setup
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel + CHANNEL_OFFSET].rx_sig, false);
}

void rx_receive() {
  int channel = CHANNEL;

  //digitalWrite(TXPIN,LOW);
  //delayMicroseconds(200);
  
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW); //Set RMT memory owner for RX channel
  rmt_ll_rx_reset_pointer(&RMT, channel); //Reset RMT reading pointer for TX channel
  rmt_ll_rx_enable(&RMT, channel, true);

  //delayMicroseconds(200);
}

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

void rx_print() {
  rx_print2("RMTMEM", RMTMEM.channels[channel + CHANNEL_OFFSET].symbols);
}

void rx_print2(String msg, void* ptr) {
  qqq_rtm_item32_t *sym = (qqq_rtm_item32_t *)ptr;
  int off = rmt_ll_rx_get_memory_writer_offset(&RMT, channel);
  Serial.print(msg);
  Serial.printf(" off=%d ",off);
  for(int i=0; i<SOC_RMT_MEM_WORDS_PER_CHANNEL;i++) {
    Serial.printf("%d%c ",(int)sym->duration0,(sym->level0?'H':'L'));
    Serial.printf("%d%c ",(int)sym->duration1,(sym->level1?'H':'L'));
    sym++;
  }
  Serial.println();
}

void rx_clear(int val) {
  val |= (val<<16);
  for(int i=0; i<SOC_RMT_MEM_WORDS_PER_CHANNEL;i++) {
    RMTMEM.channels[channel + CHANNEL_OFFSET].symbols[i].val = val;
  }
}

qqq_rtm_item32_t rmtmem_saved[SOC_RMT_MEM_WORDS_PER_CHANNEL];




void tx_setup() {
  pinMode(TXPIN,OUTPUT);
  digitalWrite(TXPIN,HIGH);  
}

void tx_send() {
  uint32_t t = esp_cpu_get_cycle_count();

  tx_send2(&t, 3, 100*240, 150*240);

  //rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_SW);
  //rmt_ll_rx_enable(&RMT, channel, true);

  memcpy(&rmtmem_saved[0],&RMTMEM.channels[channel + CHANNEL_OFFSET].symbols[0], 1*4); //save first symbol
  rmt_ll_rx_reset_pointer(&RMT, channel);
  memcpy(&rmtmem_saved[1],&RMTMEM.channels[channel + CHANNEL_OFFSET].symbols[1],SOC_RMT_MEM_WORDS_PER_CHANNEL*4 - 1*4); //save other symbols
  memset(&RMTMEM.channels[channel + CHANNEL_OFFSET].symbols[2],0,SOC_RMT_MEM_WORDS_PER_CHANNEL*4 - 2*4);

  //rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_LL_MEM_OWNER_HW);
  //rmt_ll_rx_enable(&RMT, channel, true);

  tx_send2(&t, 3, 100*240, 150*240);

  delayMicroseconds(100000);

  rx_print2("SAVED",rmtmem_saved);

}

void tx_send2(uint32_t *t,int num_pulses, uint32_t tlow, uint32_t thigh) {
  for(int i=0;i<num_pulses;i++) {
    digitalWrite(TXPIN,LOW);
    while(esp_cpu_get_cycle_count() - *t < tlow);
    *t += tlow;
    digitalWrite(TXPIN,HIGH);
    while(esp_cpu_get_cycle_count() - *t < thigh);
    *t += thigh; 
  }
}

void tx_send21(uint32_t *t,int num_pulses, uint32_t tlow, uint32_t thigh) {
  for(int i=0;i<num_pulses;i++) {
    digitalWrite(TXPIN,LOW);
    while(esp_cpu_get_cycle_count() - *t < tlow);
    *t += tlow;
    digitalWrite(TXPIN,HIGH);
    while(esp_cpu_get_cycle_count() - *t < thigh);
    *t += thigh; 
  }
  digitalWrite(TXPIN,LOW);
  while(esp_cpu_get_cycle_count() - *t < tlow);
  *t += tlow;  
}

void tx_send22(uint32_t *t,int num_pulses, uint32_t tlow, uint32_t thigh) {
  digitalWrite(TXPIN,HIGH);
  while(esp_cpu_get_cycle_count() - *t < thigh);
  *t += thigh;
  for(int i=0;i<num_pulses;i++) {
    digitalWrite(TXPIN,LOW);
    while(esp_cpu_get_cycle_count() - *t < tlow);
    *t += tlow;
    digitalWrite(TXPIN,HIGH);
    while(esp_cpu_get_cycle_count() - *t < thigh);
    *t += thigh; 
  }
}


void print_rmt() {
#if CONFIG_IDF_TARGET_ESP32S3
    Serial.printf("conf0    %08lX\n", RMT.chmconf[0].conf0.val);

    Serial.printf(" div_cnt_chm        %ld\n", RMT.chmconf[0].conf0.div_cnt_chm); 
    Serial.printf(" idle_thres_chm     %ld\n", RMT.chmconf[0].conf0.idle_thres_chm); 
    Serial.printf(" dma_access_en_chm  %ld\n", RMT.chmconf[0].conf0.dma_access_en_chm); 
    Serial.printf(" mem_size_chm       %ld\n", RMT.chmconf[0].conf0.mem_size_chm); 
    Serial.printf(" carrier_en_chm     %ld\n", RMT.chmconf[0].conf0.carrier_en_chm); 
    Serial.printf(" carrier_out_lv_chm %ld\n", RMT.chmconf[0].conf0.carrier_out_lv_chm); 
    Serial.printf(" reserved_30        %ld\n", RMT.chmconf[0].conf0.reserved_30); 

    Serial.printf("conf1    0x%08lX \n", RMT.chmconf[1].conf0.val); 
    Serial.printf("sys_conf 0x%08lX \n", RMT.sys_conf.val); 

    Serial.printf(" apb_fifo_mask      %ld\n", RMT.sys_conf.apb_fifo_mask); 
    Serial.printf(" mem_clk_force_on   %ld\n", RMT.sys_conf.mem_clk_force_on); 
    Serial.printf(" mem_force_pd       %ld\n", RMT.sys_conf.mem_force_pd); 
    Serial.printf(" mem_force_pu       %ld\n", RMT.sys_conf.mem_force_pu); 
    Serial.printf(" sclk_div_num       %ld\n", RMT.sys_conf.sclk_div_num); 
    Serial.printf(" sclk_div_a         %ld\n", RMT.sys_conf.sclk_div_a); 
    Serial.printf(" sclk_div_b         %ld\n", RMT.sys_conf.sclk_div_b); 
    Serial.printf(" sclk_sel           %ld\n", RMT.sys_conf.sclk_sel); 
    Serial.printf(" sclk_active        %ld\n", RMT.sys_conf.sclk_active); 
    Serial.printf(" reserved_27        %ld\n", RMT.sys_conf.reserved_27); 
    Serial.printf(" clk_en             %ld\n", RMT.sys_conf.clk_en); 

    Serial.printf("status   0x%08lX\n", RMT.chmstatus[0].val); 
    Serial.printf("int_raw  0x%08lX\n", RMT.int_raw.val);
    Serial.printf("int_ena  0x%08lX\n", RMT.int_ena.val);
#endif    
}