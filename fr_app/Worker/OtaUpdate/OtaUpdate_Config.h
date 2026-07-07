/*
 * OtaUpdate_Config.h
 *
 * Tunables for the OTA update orchestration: UART acquire (primary pulls the
 * firmware file from the fr9 logger) and the LoRa distribution session
 * (primary -> secondaries, direct, non-mesh).
 */

#ifndef WORKER_OTAUPDATE_OTAUPDATE_CONFIG_H_
#define WORKER_OTAUPDATE_OTAUPDATE_CONFIG_H_

/* ---- UART acquire (fr9 -> primary) ---- */
#define OTA_UART_BLOCK_LEN        1024U   /* raw file bytes per AT+FWGET pull  */
#define OTA_UART_BLOCK_RETRIES    3U      /* attempts per block                */
#define OTA_UART_RETRY_DELAY_MS   250U    /* settle time between attempts      */
#define OTA_FWREQ_WAIT_POLL_MS    2000U   /* re-poll period on FW,WAIT         */
#define OTA_FWREQ_WAIT_MAX_MS     30000U  /* give up on FW,WAIT after this     */

/* ---- LoRa distribution (primary -> secondaries) ---- */
#define OTA_LORA_CHUNK_LEN        224U    /* image payload bytes per OtaChunk
                                             (multiple of 16; chunk packet
                                             stays within the 255 B radio
                                             frame incl. header + CRC)        */
#define OTA_LORA_WINDOW_CHUNKS    64U     /* chunks per blast/repair window    */
#define OTA_LORA_MAX_TARGETS      8U      /* secondaries served per session    */
#define OTA_LORA_PREP_REPEATS     5U      /* OtaPrep announcements, 1 s apart  */
#define OTA_LORA_PREP_GAP_MS      1000U
#define OTA_LORA_CHUNK_GAP_MS     15U     /* pause between chunk transmissions */
#define OTA_LORA_POLL_TIMEOUT_MS  2500U   /* wait for one OtaReport            */
#define OTA_LORA_REPAIR_ROUNDS    3U      /* repair passes per window          */
#define OTA_LORA_RX_IDLE_MS       20000U  /* secondary: session silence abort  */
#define OTA_LORA_SESSION_MAX_MS   (12UL * 60UL * 1000UL)  /* hard session cap  */

#endif /* WORKER_OTAUPDATE_OTAUPDATE_CONFIG_H_ */
