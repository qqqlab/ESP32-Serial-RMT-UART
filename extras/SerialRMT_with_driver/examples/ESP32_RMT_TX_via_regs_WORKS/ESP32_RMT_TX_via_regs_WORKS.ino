
#include <soc/gpio_struct.h>
#include <soc/rmt_struct.h>
#include <soc/pcnt_struct.h>

//#include <soc/reg_base.h>
#include <soc/dport_reg.h>
#include <soc/rmt_reg.h>

//RMT GPIO
#include <hal/gpio_hal.h>
#include <esp_rom_gpio.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <hal/gpio_hal.h>
#include <esp_rom_gpio.h>
#include <soc/rmt_periph.h>

gpio_num_t gpio_num = (gpio_num_t)16;
uint32_t channel_id = 0;

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
} xxxrmt_item32_t;


typedef struct {
    struct {
        volatile xxxrmt_item32_t data32[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } chan[SOC_RMT_CHANNELS_PER_GROUP];
} rmt_mem_t;

extern rmt_mem_t RMTMEM;



void rm_setup() {
  //enable and reset RMT
  uint32_t en_reg = *(uint32_t *)DPORT_PERIP_CLK_EN_REG;
  if(!(en_reg & DPORT_RMT_CLK_EN)) { //if not enabled
    en_reg |= DPORT_RMT_CLK_EN;
    *(uint32_t *)DPORT_PERIP_CLK_EN_REG = en_reg; //enable RMT
    //reset RMT
    *(uint32_t *)DPORT_PERIP_RST_EN_REG = DPORT_RMT_RST;  
    *(uint32_t *)DPORT_PERIP_RST_EN_REG = 0;
  }

  Serial.printf("RMT_CH0CONF0_REG=%08lX\n", *(uint32_t *)RMT_CH0CONF0_REG);

  Serial.printf("RMT.conf0=%08lX\n", RMT.conf_ch[0].conf0.val);
  Serial.printf("RMT.conf1=%08lX\n", RMT.conf_ch[0].conf1.val);

  RMT.conf_ch[0].conf0.div_cnt = 80;         // 8bit This register is used to configure the  frequency divider's factor in channel0-7.*/
  RMT.conf_ch[0].conf0.idle_thres = 0xffff;  //16bit In receive mode when no edge is detected on the input signal for longer than reg_idle_thres_ch0 then the receive process is done.*/
  RMT.conf_ch[0].conf0.mem_size = 1;         //4bit This register is used to configure the the amount of memory blocks allocated to channel0-7.*/
  RMT.conf_ch[0].conf0.carrier_en = 0;       //This is the carrier modulation enable control bit for channel0-7.*/
  RMT.conf_ch[0].conf0.carrier_out_lv = 0;   //This bit is used to configure the way carrier wave is modulated for  channel0-7.1'b1:transmit on low output level  1'b0:transmit  on high output level.
  RMT.conf_ch[0].conf0.mem_pd = 0;           //This bit is used to reduce power consumed by memory. 1:memory is in low power state.
  //RMT.conf_ch[0].conf0.clk_en = 1;           //(reserved 0) This bit  is used  to control clock.when software configure RMT internal registers  it controls the register clock.

  RMT.conf_ch[0].conf1.tx_start = 0;        //Set this bit to start sending data for channel0-7.
  RMT.conf_ch[0].conf1.rx_en = 0;           //Set this bit to enable receiving data for channel0-7.
  RMT.conf_ch[0].conf1.mem_wr_rst = 1;      //Set this bit to reset write ram address for channel0-7 by receiver access.
  RMT.conf_ch[0].conf1.mem_wr_rst = 0;      //Set this bit to reset write ram address for channel0-7 by receiver access.
  RMT.conf_ch[0].conf1.mem_rd_rst = 1;      //Set this bit to reset read ram address for channel0-7 by transmitter access.
  RMT.conf_ch[0].conf1.mem_rd_rst = 0;      //Set this bit to reset read ram address for channel0-7 by transmitter access.
  //RMT.conf_ch[0].conf1.apb_mem_rst = 0;     //(reserved 0) Set this bit to reset W/R ram address for channel0-7 by apb fifo access (using fifo is discouraged, please see the note above at data_ch[] item)
  RMT.conf_ch[0].conf1.mem_owner = 0;       //This is the mark of channel0-7's ram usage right. 1：receiver uses the ram  0：transmitter uses the ram
  RMT.conf_ch[0].conf1.tx_conti_mode = 0;   //Set this bit to continue sending  from the first data to the last data in channel0-7 again and again.
  RMT.conf_ch[0].conf1.rx_filter_en = 0;    //This is the receive filter enable bit for channel0-7.
  RMT.conf_ch[0].conf1.rx_filter_thres = 8; //8bit in receive mode channel0-7 ignore input pulse when the pulse width is smaller then this value.
  RMT.conf_ch[0].conf1.ref_cnt_rst = 0;     //This bit is used to reset divider in channel0-7.
  RMT.conf_ch[0].conf1.ref_always_on = 1;   //This bit is used to select base clock. 1:clk_apb  0:clk_ref
  RMT.conf_ch[0].conf1.idle_out_lv = 1;     //This bit configures the output signal's level for channel0-7 in IDLE state.
  RMT.conf_ch[0].conf1.idle_out_en = 1;     //This is the output enable control bit for channel0-7 in IDLE state.

  RMT.apb_conf.fifo_mask = 1;               //RMT_MEM_ACCESS_EN Set this bit to enable RMTMEM and disable apb fifo access (using fifo is discouraged, please see the note above at data_ch[] item)


  Serial.printf("RMT.conf0=%08lX\n", RMT.conf_ch[0].conf0.val);  
  Serial.printf("RMT.conf1=%08lX\n", RMT.conf_ch[0].conf1.val);
  Serial.printf("RMT.apb_conf=%08lX\n", RMT.apb_conf.val);

  Serial.printf("MEM=%08lX\n",RMTMEM.chan[0].data32[0].val);
  RMTMEM.chan[0].data32[0].val = 0x01020304;
  Serial.printf("MEM=%08lX\n",RMTMEM.chan[0].data32[0].val);

/*
    // reserve the GPIO output path, because we don't expect another peripheral to signal to the same GPIO
    //uint64_t old_gpio_rsv_mask = esp_gpio_reserve(BIT64(gpio_num));
    // check if the GPIO is already used by others, RMT TX channel only uses the output path of the GPIO
    //if (old_gpio_rsv_mask & BIT64(gpio_num)) {
    //    ESP_LOGW(TAG, "GPIO %d is not usable, maybe conflict with others", gpio_num);
    //}
    // GPIO Matrix/MUX configuration
    //gpio_func_sel(gpio_num, PIN_FUNC_GPIO);
    gpio_ll_func_sel(&GPIO, gpio_num, PIN_FUNC_GPIO);
    // connect the signal to the GPIO by matrix, it will also enable the output path properly
    esp_rom_gpio_connect_out_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel_id].tx_sig, false, false);
*/
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO);
    //ESP_RETURN_ON_FALSE(RMT_IS_TX_CHANNEL(channel), ESP_ERR_INVALID_ARG, TAG, RMT_CHANNEL_ERROR_STR);
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel_id].tx_sig, 0, 0);
}

void rm_send()
{
  xxxrmt_item32_t item[16];
  for (int i = 0; i < 16; i++) {
      item[i].duration0 = (i+1)*100;
      item[i].level0 = 0;
      item[i].duration1 = (i+1)*100;
      item[i].level1 = 1;
  }
  item[15].duration1 = 0; //set last item duration = 0

  //copy items to RMT channel
  for (int i = 0; i < 16; i++) {
    RMTMEM.chan[channel_id].data32[i].val = item[i].val;
  }

  RMT.conf_ch[channel_id].conf1.mem_rd_rst = 1;      //Set this bit to reset read ram address for channel0-7 by transmitter access.
  RMT.conf_ch[channel_id].conf1.mem_rd_rst = 0;      //Set this bit to reset read ram address for channel0-7 by transmitter access.

  RMT.conf_ch[channel_id].conf1.tx_start = 1;        //Set this bit to start sending data for channel0-7.
  RMT.conf_ch[channel_id].conf1.tx_start = 0;        //Set this bit to start sending data for channel0-7.
}

void setup() {
  Serial.begin(115200);

  rm_setup();
}


void loop() {
  rm_send();
  delay(200);
}





#if 0

/*
  Serial.printf("GPIO.in =%08lX\n",GPIO.in);
  Serial.printf("GPIO.out=%08lX\n",GPIO.out);  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  Serial.printf("GPIO.in =%08lX\n",GPIO.in);
  Serial.printf("GPIO.out=%08lX\n",GPIO.out);
  uint32_t adr = GPIO_OUT_REG;
  uint32_t *p = (uint32_t *)adr;
  Serial.printf("GPIO_OUT_REG=%08lX\n", *p); 
  Serial.printf("GPIO_OUT_REG=%08lX\n", *(uint32_t *)GPIO_OUT_REG); 

  for(int i=0;i<=0x40;i+=4) {
    Serial.printf("GPIO_BASE[0x%04X]=%08lX\n", i, *(uint32_t *)(DR_REG_GPIO_BASE + i));
  }

  for(int i=0;i<2;i++) {
    Serial.printf("DPORT[0x%04X]=%08lX\n", i, *(uint32_t *)(0x3FF000C0 + i*4));
  }

  for(int i=0;i<6;i++) {
    Serial.printf("DPORT[0x%04X]=%08lX\n", i, *(uint32_t *)((DR_REG_DPORT_BASE + 0x0C0) + i*4));
  }
*/




//handle transmission completed interrupt
IRAM_ATTR void rm_tx_end_handler() {
  cnt++;
}

//RMT Interrupt Handler
IRAM_ATTR void rmt_isr_handler(void* arg){
  uint32_t status;

  //TX end interrupt 
  status = rmt_ll_get_tx_end_interrupt_status(&RMT);
  for(int channel=0; channel<SOC_RMT_TX_CANDIDATES_PER_GROUP; channel++) {
    if (status & (1<<channel) ) {
      rm_tx_end_handler();
      rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(channel)); //IDF 5.4
      //rmt_ll_clear_tx_end_interrupt(&RMT, ch); //IDF 4.4
      //RMT.int_clr.val = (1 << (channel * 3));; //ESP32

    }
  }
}

#endif