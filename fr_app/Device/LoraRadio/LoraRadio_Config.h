/*
 * LoraRadio_Config.h
 *
 * LoRa radio configuration — frequency, RF parameters and packet size.
 */

#ifndef DEVICE_LORARADIO_LORARADIO_CONFIG_H_
#define DEVICE_LORARADIO_LORARADIO_CONFIG_H_

#define RF_FREQUENCY                868000000   /* Hz        */
#define TX_OUTPUT_POWER             15          /* dBm       */
#define LORA_BANDWIDTH              LORA_BW_250 /* kHz       */
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1
#define LORA_PREAMBLE_LENGTH        8           /* Tx and Rx */
#define LORA_SYMBOL_TIMEOUT         5           /* Symbols   */
#define LORA_SYNCWORD               0x1424

#define LORA_MAX_PACKET_SIZE        (256)

#endif /* DEVICE_LORARADIO_LORARADIO_CONFIG_H_ */
