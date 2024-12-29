#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_RX_GPIO_NUM       17

static const char *TAG = "rmt_rx";

// save the received RMT symbols
static rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame

static rmt_rx_done_event_data_t rx_data;
static QueueHandle_t receive_queue;
static rmt_channel_handle_t rx_channel = NULL;
static rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void rx_setup() {
    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

    ESP_LOGI(TAG, "register RX done callback");
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    ESP_LOGI(TAG, "enable RMT RX channels");
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
}

int rx_receive(uint32_t* buf, int size) {
    // wait for RX done signal
    if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
        // parse the receive symbols and print the result
        //example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
        int len = rx_data.num_symbols;
        if(len>size) len=size;
        memcpy(buf, rx_data.received_symbols, len*4);
        // start receive again
        ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        return len;
    } else {
        // timeout, transmit predefined IR NEC packets
        return -1;
    }
}
