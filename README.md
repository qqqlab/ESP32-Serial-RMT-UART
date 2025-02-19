# ESP32-Serial-RMT-UART

Additional hardware serial ports for ESP32 family. This is a hardware UART based on the RMT peripheral. It is a drop-in replacement for standard ESP32 Arduino Serial class.

## Supported Chips

| Chip | TX | RX | Additional Serial ports
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

*SerialRMT.ino Test Results*

|SerialRMT | Result | 
|-|-|
ESP32S3_TX | PASS 300 to 8000000 (8M) baud
ESP32S3_RX | PASS 300 to 2000000 (2M) baud
ESP32_TX   | FAIL

Note: only ESP32 and ESP32-S3 were actually tested by me, but the others should work based on the hardware descriptions.


## Requirements

- [ESP-IDF](https://github.com/espressif/esp-idf) version 5.0 or later 
- -or- [arduino-esp32](https://github.com/espressif/arduino-esp32/) version 3.0 or later (which is based on ESP-IDF 5)

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

## Software Details

The SerialRMT class does not implement the full set of Arduino Serial functions. Notable omission is end(). I use this project to call SerialRMT.begin() once to setup the UART, and then use it unmodified. If you need to change baud, pins, etc. during runtime, please add the corresponding functions.

The SerialRMT class is not thread-safe. Only one task should transmit data, and only one task should receive data. The transmit and receive tasks can be two different tasks.

### RMT UART Transmitter

The RMT transmitter sends out pulses, and has a buffer for 96 high or low pulses. SerialRMT writes chunks of up to 9 UART bytes (10 pulses per byte) to the RMT transmitter buffer and starts the RMT transmitter. An interrupt is generated when the buffer is transmitted, and the process repeats.

### RMT UART Receiver

The RMT receiver stores the duration and high/low level of up to 96 received UART pulses in a buffer. When the buffer is half full or full a threshold interrupt is generated, and the pulses in the upper or lower half of the buffer are decoded. Also, the buffer wraps around to the start, to allow for continuous operation. But unfortunately the RMT receiver stops after a certain timeout, which would make it basically useless as an UART. This timeout generates the end interrupt, which is used to quickly restart the RMT receiver. As it turns out however, that no pulses are lost, because the RMT receiver still counts the current pulse during the "stopped" period, and thus the correct pulse duration will be written to the buffer after restart.

## Hardware Details

The Remote Control Peripheral (RMT) is a pulse capture/sending unit.

This project uses the RMT_LL (RTM low level) interface of ESP-IDF version 5.0. For later or earlier ESP-IDF version some adaptions are required, as the RMT_LL interface changes with each ESP-IDF version. Adaptions might be needed for other targets than the tested ESP32-S3, as the RMT_LL interface uses different #define constants.

Two hardware RMT versions exist. The RMT version 1 receiver always stops after filling the receive buffer. The newer RMT version 2 receiver has SOC_RMT_SUPPORT_RX_PINGPONG capability, which means that the receive buffer can wrap around to the start. So, only the newer version allows continuous operation needed for UART RX. 

It might be possible to pimp the older RMT-v1 hardware by using the Pulse Count Controller (PCNT) to generate an interrupt when the RMT receive buffer is half full. In the PCNT interrupt handler, copy the first half of the RMT buffer whilst RMT keeps running, then trigger RMT_MEM_WR_RST_CHn to wrap RMT write pointer to the start of the buffer. Now copy the second half of the RMT buffer, then clear the second half of RMT buffer. Decode the copied buffer to achieve "continuous" operation. Maybe a future project ;-)

In the extras folder there is an unsupported/unfinished version of SerialRMT based on the ESP-IDFv5 RMT driver. I found out that the RMT driver radically changed between ESP-IDFv4 and ESP-IDFv5. As the changes where not a real improvement for this project, I decided to go more bare-metal and base the project on RMT_LL in the hope that it will survive more ESP-IDF version iterations.

Information about ESP-IDF hardware abstaction can be found [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/hardware-abstraction.html).

The capabilities of the different RMT versions listed below can be found on https://github.com/espressif/esp-idf in folder `esp-idf/components/soc/esp32***/include/soc/soc_caps.h`. RMT-v1 and RMT-v2 are not official designators, and it appears that each chip uses a slightly different RMT implementation.

```
//=====================================================================
// ESP32 -- RMT-v1
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
// ESP32-C3 -- RMT-v2
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
// ESP32-C5 -- RMT-v2
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
// ESP32-C6 -- RMT-v2
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
// ESP32-H2 -- RMT-v2
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
// ESP32-P4 -- RMT-v2
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
// ESP32-S2 -- RMT-v1
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
// ESP32-S3 -- RMT-v2
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
