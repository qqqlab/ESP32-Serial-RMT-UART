/*RMT RX with <driver/rmt_rx.h>
2024-12-27 tested with: 
ESP32 and ESP32S3
ESP-IDF Version  v5.3.2-174-g083aad99cf-dirty
Arduino Version  3.1.0

connect TXPIN to EXAMPLE_IR_RX_GPIO_NUM
*/

#define TXPIN 16

#include "rx.h"

#include <soc/rmt_struct.h>



void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("TARGET: " CONFIG_IDF_TARGET);

  tx_setup();
  rx_setup();
}


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The layout of RMT symbol stored in memory, which is decided by the hardware design
 */
typedef union {
    struct {
        uint16_t duration0 : 15; /*!< Duration of level0 */
        uint16_t level0 : 1;     /*!< Level of the first part */
        uint16_t duration1 : 15; /*!< Duration of level1 */
        uint16_t level1 : 1;     /*!< Level of the second part */
    };
    uint32_t val; /*!< Equivalent unsigned value for the RMT symbol */
} qqq_rmt_symbol_word_t;


extern qqq_rmt_symbol_word_t RMTMEM[48*8];


#ifdef __cplusplus
}
#endif
/*

void qqq_rmt_print(qqq_rmt_symbol_word_t* symbols, int sym_cnt) {
  for(int i=0; i<sym_cnt;i++) {
    qqq_rmt_symbol_word_t sym = symbols[i];
    Serial.printf("%d%c ",(int)sym.duration0,(sym.level0?'H':'L'));
    Serial.printf("%d%c ",(int)sym.duration1,(sym.level1?'H':'L'));
    if(i%10==9) Serial.println();
  }
}
*/
qqq_rmt_symbol_word_t buf[500];

int cnt = 0;

void loop() {
  int sym_cnt = rx_receive((uint32_t*)buf,sizeof(buf)/4);
  if(sym_cnt>=0) {
    Serial.println("RMT_RX_DRIVER " CONFIG_IDF_TARGET);
    Serial.printf("\nrx sym_cnt=%d\n",sym_cnt);

    Serial.print("DRIVER:");
    for(int i=0; i<sym_cnt;i++) {
      qqq_rmt_symbol_word_t sym = buf[i];
      Serial.printf("%d%c ",(int)sym.duration0,(sym.level0?'H':'L'));
      Serial.printf("%d%c ",(int)sym.duration1,(sym.level1?'H':'L'));
    }
    Serial.println();

    cnt++;
    Serial.print("RMTMEM:");
    for(int i=0; i<7;i++) {
      qqq_rmt_symbol_word_t sym = RMTMEM[48*4+i];
      Serial.printf("%d%c ",(int)sym.duration0,(sym.level0?'H':'L'));
      Serial.printf("%d%c ",(int)sym.duration1,(sym.level1?'H':'L'));
      RMTMEM[48*4+i].val = cnt;
    }
    Serial.println();   

    print_rmt();

  }else{
    tx_send();
  }
}










void tx_setup() {
  pinMode(TXPIN,OUTPUT);
  digitalWrite(TXPIN,HIGH);  
}

void tx_send() {
  uint32_t t = esp_cpu_get_cycle_count(); 
  for(int i=0;i<5;i++) {
    digitalWrite(TXPIN,LOW);
    while(esp_cpu_get_cycle_count() - t < 240*(i*300+100));
    digitalWrite(TXPIN,HIGH);
    while(esp_cpu_get_cycle_count() - t < 240*(i*300+300));    
  }
  delayMicroseconds(100000);
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

