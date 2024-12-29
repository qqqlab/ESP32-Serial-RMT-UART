# ESP32-Serial-RMT-UART

NOTE: This is an unsupported/unfinished version of SerialRMT based on the ESP-IDF version 5 RMT Driver (driver/rmt_tx.h and driver/rmt_rx.h)

Hardware UART based on the RMT peripheral. Add extra UARTs to your project, drop-in replacement for standard Arduino Serial.

## Supported Chips

| Chip | TX | RX | Number of RMT-UARTs
|:-:|:-:|:-:|-|
ESP32 | :white_check_mark: | :x: | 4
ESP32-C2 |  :x: | :x: | None
ESP32-C3 | :white_check_mark: | :white_check_mark: | 2
ESP32-C5 | :white_check_mark: | :white_check_mark: | 2
ESP32-C6 | :white_check_mark: | :white_check_mark: | 2
ESP32-C61 |  :x: | :x: | None
ESP32-H2 | :white_check_mark: | :white_check_mark: | 2
ESP32-H21 |  :x: | :x: | None
ESP32-P4 | :white_check_mark: | :white_check_mark: | 4
ESP32-S2 | :white_check_mark: | :x: | 2
ESP32-S3 | :white_check_mark: | :white_check_mark: | 4

Note: only ESP32 and ESP32-S3 were actually tested by me, but the others should work based on the hardware descriptions.

## Requirements

- ESP-IDF version 5.0 or later 
- -or- arduino-esp32 version 3.0 or later (which is based on ESP-IDF 5)

## Usage

```
#include <ESP32_SerialRMT.h>

SerialRMT serialRMT;

void setupESP32() {
  serialRMT.begin(115200, 4, 5); //baud, rxPin, txPin - set pin to -1 if not used
}

void loop() {
  serialRMT.print("blablabla");

  if(serialRMT.available() > 0) {
    char x = serialRMT.read();
  }
}
```
## FAQ

### ESP32:  E (10) rmt: rmt_receive(366): partial receive not supported

You're using an older chip which does not support RX, set rxPin to -1.

## Software Details

The SerialRMT class does not implement the full set of Arduino Serial functions. Notable omission is end(). I use this project to call SerialRMT.begin() once to setup the UART, and then use it unmodified. If you need to change baud, pins, etc. during runtime, please add the corresponding functions.

## Hardware Details

The Remote Control Peripheral (RMT) is a pulse capture/sending unit.

This project uses the new RMT driver introduced in ESP-IDF version 5.0.

Two hardware RMT versions exist. The older RMT receiver always stops after filling the receive buffer. The newer RMT receiver has SOC_RMT_SUPPORT_RX_PINGPONG capability, which means that the receive buffer can wrap around to the start. So, only the newer version allows continuous operation needed for UART RX. 

I think it is possible to pimp the older RMT hardware by using the Pulse Count Controller (PCNT) to generate an interrupt when the RMT receive buffer is half full. In the PCNT interrupt handler, copy the RMT buffer whilst RMT keeps running, then trigger RMT_MEM_WR_RST_CHn to wrap RMT to the start, and copy the RMT buffer again. Use software to stitch the two RMT receive buffers together to achieve "continuous" operation. Maybe a future project ;-)

The capabilities of the different chips can be found on https://github.com/espressif/esp-idf in folder `esp-idf/components/soc/esp32***/include/soc/soc_caps.h`

```
//=====================================================================
// ESP32
//=====================================================================
#define SOC_RMT_GROUPS                  1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP 8  /*!< Number of channels that capable of Transmit in each group */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP 8  /*!< Number of channels that capable of Receive in each group */
#define SOC_RMT_CHANNELS_PER_GROUP      8  /*!< Total 8 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL   64 /*!< Each channel owns 64 words memory */
#define SOC_RMT_SUPPORT_REF_TICK        1  /*!< Support set REF_TICK as the RMT clock source */
#define SOC_RMT_SUPPORT_APB             1  /*!< Support set APB as the RMT clock source */
#define SOC_RMT_CHANNEL_CLK_INDEPENDENT 1  /*!< Can select different source clock for each channel */


//=====================================================================
// ESP32-C2 -- NO RMT SUPPORT
//=====================================================================


//=====================================================================
// ESP32-C3
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Transmit */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Receive */
#define SOC_RMT_CHANNELS_PER_GROUP            4  /*!< Total 4 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_APB                   1  /*!< Support set APB as the RMT clock source */
#define SOC_RMT_SUPPORT_RC_FAST               1  /*!< Support set RC_FAST clock as the RMT clock source */


//=====================================================================
// ESP32-C5
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Transmit */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Receive */
#define SOC_RMT_CHANNELS_PER_GROUP            4  /*!< Total 4 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP     1  /*!< Hardware support of auto-stop in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_SLEEP_RETENTION       1  /*!< The sleep retention feature can help back up RMT registers before sleep */


//=====================================================================
// ESP32-C6
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Transmit */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Receive */
#define SOC_RMT_CHANNELS_PER_GROUP            4  /*!< Total 4 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP     1  /*!< Hardware support of auto-stop in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_RC_FAST               1  /*!< Support set RC_FAST as the RMT clock source */
#define SOC_RMT_SUPPORT_SLEEP_RETENTION       1  /*!< The sleep retention feature can help back up RMT registers before sleep */


//=====================================================================
// ESP32-C61 -- NO RMT SUPPORT
//=====================================================================


//=====================================================================
// ESP32-H2
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Transmit */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       2  /*!< Number of channels that capable of Receive */
#define SOC_RMT_CHANNELS_PER_GROUP            4  /*!< Total 4 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP     1  /*!< Hardware support of auto-stop in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_RC_FAST               1  /*!< Support set RC_FAST as the RMT clock source */
#define SOC_RMT_SUPPORT_SLEEP_RETENTION       1  /*!< The sleep retention feature can help back up RMT registers before sleep */


//=====================================================================
// ESP32-H21 -- NO RMT SUPPORT
//=====================================================================


//=====================================================================
// ESP32-P4
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Transmit in each group */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Receive in each group */
#define SOC_RMT_CHANNELS_PER_GROUP            8  /*!< Total 8 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP     1  /*!< Hardware support of auto-stop in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_RC_FAST               1  /*!< Support set RC_FAST clock as the RMT clock source */
#define SOC_RMT_SUPPORT_DMA                   1  /*!< RMT peripheral can connect to DMA channel */
#define SOC_RMT_SUPPORT_SLEEP_RETENTION       1  /*!< The sleep retention feature can help back up RMT registers before sleep */


//=====================================================================
// ESP32-S2
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Transmit in each group */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Receive in each group */
#define SOC_RMT_CHANNELS_PER_GROUP            4  /*!< Total 4 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         64 /*!< Each channel owns 64 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmitting specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_REF_TICK              1  /*!< Support set REF_TICK as the RMT clock source */
#define SOC_RMT_SUPPORT_APB                   1  /*!< Support set APB as the RMT clock source */
#define SOC_RMT_CHANNEL_CLK_INDEPENDENT       1  /*!< Can select different source clock for each channel */


//=====================================================================
// ESP32-S3
//=====================================================================
#define SOC_RMT_GROUPS                        1U /*!< One RMT group */
#define SOC_RMT_TX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Transmit in each group */
#define SOC_RMT_RX_CANDIDATES_PER_GROUP       4  /*!< Number of channels that capable of Receive in each group */
#define SOC_RMT_CHANNELS_PER_GROUP            8  /*!< Total 8 channels */
#define SOC_RMT_MEM_WORDS_PER_CHANNEL         48 /*!< Each channel owns 48 words memory (1 word = 4 Bytes) */
#define SOC_RMT_SUPPORT_RX_PINGPONG           1  /*!< Support Ping-Pong mode on RX path */
#define SOC_RMT_SUPPORT_RX_DEMODULATION       1  /*!< Support signal demodulation on RX path (i.e. remove carrier) */
#define SOC_RMT_SUPPORT_TX_ASYNC_STOP         1  /*!< Support stop transmission asynchronously */
#define SOC_RMT_SUPPORT_TX_LOOP_COUNT         1  /*!< Support transmit specified number of cycles in loop mode */
#define SOC_RMT_SUPPORT_TX_LOOP_AUTO_STOP     1  /*!< Hardware support of auto-stop in loop mode */
#define SOC_RMT_SUPPORT_TX_SYNCHRO            1  /*!< Support coordinate a group of TX channels to start simultaneously */
#define SOC_RMT_SUPPORT_TX_CARRIER_DATA_ONLY  1  /*!< TX carrier can be modulated to data phase only */
#define SOC_RMT_SUPPORT_XTAL                  1  /*!< Support set XTAL clock as the RMT clock source */
#define SOC_RMT_SUPPORT_RC_FAST               1  /*!< Support set RC_FAST clock as the RMT clock source */
#define SOC_RMT_SUPPORT_APB                   1  /*!< Support set APB as the RMT clock source */
#define SOC_RMT_SUPPORT_DMA                   1  /*!< RMT peripheral can connect to DMA channel */
```