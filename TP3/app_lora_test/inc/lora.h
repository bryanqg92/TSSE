#include "api_lora.h"
#include <stdint.h>

#define LORA_SF_7 0x07
#define LORA_SF_8 0x08
#define LORA_SF_9 0x09
#define LORA_SF_10 0x0A
#define LORA_SF_11 0x0B
#define LORA_SF_12 0x0C
#define LORA_BW 4 // 125E3
#define LORA_FREQ 433E6
#define LORA_TX_POWER 22
#define LORA_PREAMBLE_LENGTH 8
#define LORA_PAYLOAD_LENGTH 0
#define LORA_CODING_RATE 0x01
#define LORA_SYNC_WORD 0x34
#define LORA_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define LORA_CMD_CALIBRATE_IMAGE 0x98
#define LORA_CRC_ON true
#define LORA_TXCO_VOLTAGE 0



typedef struct {
    uint16_t nitrogen;
    uint16_t phosphorus;
    uint16_t potassium;
    float temperature;
    float moisture;
    float pH;
    uint16_t conductivity;
} SoilData_t;

typedef struct {
    float altitude;
    float latitude;
    float longitude;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
} GNSSData_t;

void lora_init(void);

void task_lora_rx(void);
