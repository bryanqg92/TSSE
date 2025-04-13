#include "lora.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "logger.h"

static char* TAG = "==> [APP_LORA]";

void decode_lora_data(uint8_t* data_buffer, SoilData_t* soilData, GNSSData_t* gnssData, uint8_t* category) {
	uint8_t offset = 0;

	// SoilData
	memcpy(&soilData->nitrogen, &data_buffer[offset], 2);
	offset += 2;
	memcpy(&soilData->phosphorus, &data_buffer[offset], 2);
	offset += 2;
	memcpy(&soilData->potassium, &data_buffer[offset], 2);
	offset += 2;
	memcpy(&soilData->temperature, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&soilData->moisture, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&soilData->pH, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&soilData->conductivity, &data_buffer[offset], 2);
	offset += 2;

	// GNSSData
	memcpy(&gnssData->altitude, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&gnssData->latitude, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&gnssData->longitude, &data_buffer[offset], 4);
	offset += 4;
	memcpy(&gnssData->year, &data_buffer[offset], 2);
	offset += 2;
	memcpy(&gnssData->month, &data_buffer[offset], 1);
	offset += 1;
	memcpy(&gnssData->day, &data_buffer[offset], 1);
	offset += 1;
	memcpy(&gnssData->hour, &data_buffer[offset], 1);
	offset += 1;
	memcpy(&gnssData->minute, &data_buffer[offset], 1);
	offset += 1;

	*category = data_buffer[offset];
}


void lora_init(void) {

	//LoRaDebugPrint(false);
	if (LoRaBegin(LORA_FREQ, LORA_TX_POWER, LORA_TXCO_VOLTAGE, false) != 0)
	{
		STM_LOGE(TAG, "Failed to recognize the LoRa module");
		return;
	}
	else
	{
		LoRaConfig(LORA_SF_10, SX126X_LORA_BW_125_0, SX126X_LORA_CR_4_6, LORA_PREAMBLE_LENGTH,
				   LORA_PAYLOAD_LENGTH, SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_STANDARD);
	}
	STM_LOGI(TAG, "Init done");


}


void task_lora_rx(){

    uint8_t buffer[sizeof(SoilData_t) + sizeof(GNSSData_t) + 1];
    SoilData_t soilData;
    GNSSData_t gnssData;
    uint8_t category;

	uint8_t len = LoRaReceive(buffer, sizeof(buffer));

	if (len > 0) {
		
		int8_t rssi, snr;
		GetPacketStatus(&rssi, &snr);
		decode_lora_data(buffer, &soilData, &gnssData, &category);

		STM_LOGD(TAG, "Soil Data: N:%d P:%d K:%d T:%.2f M:%.2f pH:%.2f C:%d\n",
				soilData.nitrogen, soilData.phosphorus, soilData.potassium,
				soilData.temperature, soilData.moisture, soilData.pH,
				"RSSI: %d dBm, SNR: %d dB\n"soilData.conductivity);
		STM_LOGD(TAG, "GNSS Data: Lat:%.6f Lon:%.6f Alt:%.2f\n",
				gnssData.latitude, gnssData.longitude, gnssData.altitude);
		
		STM_LOGD(TAG, "Date: %04d-%02d-%02d %02d:%02d\n",
				gnssData.year, gnssData.month, gnssData.day,
				gnssData.hour, gnssData.minute);
		STM_LOGD(TAG, , rssi, snr);
		
		STM_LOGD(TAG, "category: %d\n", category);
	}
	else{
		STM_LOGD(TAG, "No data received");
	}

}







