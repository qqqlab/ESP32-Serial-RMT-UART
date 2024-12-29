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
#pragma once

#include <soc/soc_caps.h> 

/*
NOTE: there are 3 channel numbering schemes: DATASHEET, RMT_LL, and Rmt_uart
for TX they are all the same
for RX it depends on the RMT version: 
                      ESP32  ESP32-S3  
RX channel Datasheet   4-7     4-7
RX channel RMT_LL      4-7     0-3
RX channel Rmt_uart    0-3     0-3
*/
#if SOC_RMT_SUPPORT_RX_PINGPONG && (SOC_RMT_CHANNELS_PER_GROUP == (2 * SOC_RMT_RX_CANDIDATES_PER_GROUP)) && (SOC_RMT_CHANNELS_PER_GROUP == (2 * SOC_RMT_TX_CANDIDATES_PER_GROUP))
  //RMT-v2 (ESP32S3, ESP32C6, etc)
  //RMT-v2 has specific TX and RX RMT channels, with PINGPONG
  //in datasheet channels (n) 0-3 are TX and channels (m) 4-7 are RX channels
  //in RMT_LL both TX and RX channels are numbered 0-3, so for RTMMEM RAM (rx_ll_channel + QQQ_RX_CHANNEL_RTMMEM_OFFSET) is needed to access
  #define QQQ_RMT_VERSION 2
  #define QQQ_TX_CHANNELS (SOC_RMT_TX_CANDIDATES_PER_GROUP)
  #define QQQ_RX_CHANNELS (SOC_RMT_RX_CANDIDATES_PER_GROUP) 
  #define QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET QQQ_TX_CHANNELS // {datasheet channel} = {rtm_ll channel} + QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET
  #define QQQ_RX_CHANNEL_UART_TO_LL_OFFSET 0 //{rtm_ll channel} = (rx[] uart index} + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET
#elif !SOC_RMT_SUPPORT_RX_PINGPONG && (SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_RX_CANDIDATES_PER_GROUP) && (SOC_RMT_CHANNELS_PER_GROUP == SOC_RMT_TX_CANDIDATES_PER_GROUP)
  //RMT-v1 (ESP32, ESP32S2, etc)
  //RMT-v1 has channels that can be used as TX or RX, without PINGPONG
  //We use lower half for TX, higher half for RX, so QQQ_RX_RTMMEM_OFFSET = 0
  #define QQQ_RMT_VERSION 1  
  #define QQQ_TX_CHANNELS ((SOC_RMT_TX_CANDIDATES_PER_GROUP)/2)
  #define QQQ_RX_CHANNELS ((SOC_RMT_RX_CANDIDATES_PER_GROUP)/2)  
  #define QQQ_RX_CHANNEL_LL_TO_DATASHEET_OFFSET 0 //{datasheet channel} = {rtm_ll channel} + QQQ_RX_CHANNEL_RTMMEM_OFFSET
  #define QQQ_RX_CHANNEL_UART_TO_LL_OFFSET QQQ_TX_CHANNELS //{rtm_ll channel} = (rx[] index} + QQQ_RX_CHANNEL_UART_TO_LL_OFFSET
#else
  #error "Unknown RMT version"
#endif
