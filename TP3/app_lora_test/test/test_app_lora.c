/************************************************************************************************
Copyright (c) 2025, Leandro Quiroga <bryanqg92@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
*************************************************************************************************/

/** @file test_app_lora.c
 ** @brief 
 **/

/* === Headers files inclusions =============================================================== */
#include "mock_api_lora.h"
#include "mock_logger.h"
#include "lora.h"
#include "unity.h"

/* === Macros definitions ====================================================================== */
/**
 * @brief Definiciones de valores hexadecimales para pruebas de datos de suelo
 *
 * @details Estos macros contienen valores hexadecimales para pruebas de
 * diferentes parámetros del suelo:
 *
 * - TEST_NITROGEN_HEX:     Valor 0x0064 (100 decimal) como uint16_t
 * - TEST_PHOSPHORUS_HEX:   Valor 0x00C8 (200 decimal) como uint16_t
 * - TEST_POTASSIUM_HEX:    Valor 0x012C (300 decimal) como uint16_t
 * - TEST_TEMPERATURE_HEX:  Valor 0x41CC0000 (25.5) como float
 * - TEST_MOISTURE_HEX:     Valor 0x42480000 (50.0) como float
 * - TEST_PH_HEX:           Valor 0x40E00000 (7.0) como float
 * - TEST_CONDUCTIVITY_HEX: Valor 0x05DC (1500 decimal) como uint16_t
 */

#define TEST_NITROGEN_HEX 0x64, 0x00
#define TEST_PHOSPHORUS_HEX 0xC8, 0x00
#define TEST_POTASSIUM_HEX 0x2C, 0x01
#define TEST_TEMPERATURE_HEX 0x00, 0x00, 0xCC, 0x41
#define TEST_MOISTURE_HEX 0x00, 0x00, 0x48, 0x42
#define TEST_PH_HEX 0x00, 0x00, 0xE0, 0x40
#define TEST_CONDUCTIVITY_HEX 0xDC, 0x05
/**
 * @defgroup TEST_GEO_TIME_CONSTANTS Constantes para pruebas de geolocalización
 * y tiempo
 * @brief Conjunto de macros con valores hexadecimales para pruebas
 *
 * @details Estos macros contienen valores hexadecimales para pruebas de
 * geolocalización y tiempo:
 *
 * - TEST_ALTITUDE_HEX:    Valor 0x42C80000 (100.0f como float)
 * - TEST_LATITUDE_HEX:    Valor 0xC2080000 (-34.0f como float)
 * - TEST_LONGITUDE_HEX:   Valor 0xC2680000 (-58.0f como float)
 * - TEST_YEAR_HEX:        Valor 0x07E7 (2023 como uint16_t)
 * - TEST_MONTH_HEX:       Valor 0x0A (10 como uint8_t, octubre)
 * - TEST_DAY_HEX:         Valor 0x01 (1 como uint8_t)
 * - TEST_HOUR_HEX:        Valor 0x0C (12 como uint8_t)
 * - TEST_MINUTE_HEX:      Valor 0x1E (30 como uint8_t)
 * - TEST_CATEGORY_HEX:    Valor 0x01 (1 como uint8_t)
 * @{
 */
#define TEST_ALTITUDE_HEX         0x00, 0x00, 0xC8, 0x42
#define TEST_LATITUDE_HEX         0x00, 0x00, 0x08, 0xC2
#define TEST_LONGITUDE_HEX        0x00, 0x00, 0x68, 0xC2
#define TEST_YEAR_HEX             0xE7, 0x07
#define TEST_MONTH_HEX            0x0A
#define TEST_DAY_HEX              0x01
#define TEST_HOUR_HEX             0x0C
#define TEST_MINUTE_HEX           0x1E
#define TEST_CATEGORY_HEX         0x01

#define NO_DATA_RECEIVED 0
#define DATA_RECEIVED_LEN 39

#define NO_LDO_REGULATOR false
/* === Private data type declarations ========================================================== */
static uint16_t *port_adress;

static uint8_t TEST_LORA_BUFFER_HEX[] = {
    TEST_NITROGEN_HEX,
    TEST_PHOSPHORUS_HEX,
    TEST_POTASSIUM_HEX,
    TEST_TEMPERATURE_HEX,
    TEST_MOISTURE_HEX,
    TEST_PH_HEX,
    TEST_CONDUCTIVITY_HEX,
    TEST_ALTITUDE_HEX,
    TEST_LATITUDE_HEX,
    TEST_LONGITUDE_HEX,
    TEST_YEAR_HEX,
    TEST_MONTH_HEX,
    TEST_DAY_HEX,
    TEST_HOUR_HEX,
    TEST_MINUTE_HEX,
    TEST_CATEGORY_HEX
};
/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */


/* === Public function implementation ========================================================== */

void setUp(void) {}
void tearDown(void) {}

void test_lora_init_falla_logger_error(void)
{
    LoRaBegin_ExpectAndReturn(LORA_FREQ, LORA_TX_POWER, LORA_TXCO_VOLTAGE, NO_LDO_REGULATOR, 1);
    logger_log_Expect(LOG_LEVEL_ERROR, "Failed to recognize the LoRa module");

    lora_init(); 
}

void test_lora_init_ok_configura_y_loggea_info(void)
{
    LoRaBegin_ExpectAndReturn(LORA_FREQ, LORA_TX_POWER, LORA_TXCO_VOLTAGE, NO_LDO_REGULATOR, 0);
    LoRaConfig_Expect(LORA_SF_10, SX126X_LORA_BW_125_0, SX126X_LORA_CR_4_6,
                      LORA_PREAMBLE_LENGTH, LORA_PAYLOAD_LENGTH,
                      SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_STANDARD);

    logger_log_Expect(LOG_LEVEL_INFO, "Init done");
    lora_init();
}

/**
 * @brief Prueba para la funcionalidad de decodificación de datos LoRa
 *
 * @details Esta prueba verifica que la función decode_lora_data extraiga
 * correctamente datos de sensores del suelo, información de posicionamiento
 * GNSS y datos de categoría de un buffer de paquete LoRa. Utiliza un buffer de
 * prueba predefinido (TEST_LORA_BUFFER_HEX) y comprueba que todos los campos se
 * extraigan correctamente con sus valores esperados:
 * - Datos del suelo: nitrógeno (100), fósforo (200), potasio (300), temperatura
 * (25.5°C), humedad (50.0%), pH (7.0), conductividad (1500)
 * - Datos GNSS: altitud (100.0m), coordenadas (-34.0, -58.0), fecha
 * (2023-10-01), hora (12:30)
 * - Categoría: 1
 *
 * @test Valida la función decode_lora_data
 */

void test_decode_lora_data(void)
{
    uint8_t *data_buffer = TEST_LORA_BUFFER_HEX;
    SoilData_t soilData;
    GNSSData_t gnssData;
    uint8_t category;

    decode_lora_data(data_buffer, &soilData, &gnssData, &category);

    TEST_ASSERT_EQUAL_UINT16(soilData.nitrogen, 100);
    TEST_ASSERT_EQUAL_UINT16(soilData.phosphorus, 200);
    TEST_ASSERT_EQUAL_UINT16(soilData.potassium, 300);
    TEST_ASSERT_EQUAL_FLOAT(soilData.temperature, 25.5f);
    TEST_ASSERT_EQUAL_FLOAT(soilData.moisture, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(soilData.pH, 7.0f);
    TEST_ASSERT_EQUAL_UINT16(soilData.conductivity, 1500);
    TEST_ASSERT_EQUAL_FLOAT(gnssData.altitude, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(gnssData.latitude, -34.0f);
    TEST_ASSERT_EQUAL_FLOAT(gnssData.longitude, -58.0f);
    TEST_ASSERT_EQUAL_UINT16(gnssData.year, 2023);
    TEST_ASSERT_EQUAL_UINT8(gnssData.month, 10);
    TEST_ASSERT_EQUAL_UINT8(gnssData.day, 1);
    TEST_ASSERT_EQUAL_UINT8(gnssData.hour, 12);
    TEST_ASSERT_EQUAL_UINT8(gnssData.minute, 30);
    TEST_ASSERT_EQUAL_UINT8(category, 1);
    
}

/**
 * @brief Prueba que verifica el comportamiento de task_lora_rx cuando no hay
 * datos recibidos
 *
 * @details Esta prueba valida que cuando LoRaReceive devuelve NO_DATA_RECEIVED,
 *          la función task_lora_rx debe registrar un mensaje de depuración
 *          indicando que no se recibieron datos.
 *
 * @test Verifica la correcta gestión cuando no hay datos disponibles
 */
void test_task_lora_rx_should_not_receive_data(void)
{
    LoRaReceive_IgnoreAndReturn(NO_DATA_RECEIVED);   
    logger_log_Expect(LOG_LEVEL_DEBUG,"No data received");
    task_lora_rx();
}

/**
 * @brief Prueba unitaria para verificar la recepción de datos LoRa
 * 
 * @details Esta prueba verifica que la función task_lora_rx procese correctamente
 *          los datos recibidos a través de LoRa. Simula la recepción de un paquete
 *          de datos que contiene información de sensores, datos GNSS y fecha.
 * 
 * @pre Se configura LoRaReceive para devolver DATA_RECEIVED_LEN
 * @pre Se ignora la llamada a GetPacketStatus
 * 
 * @expected La función debe llamar a logger_log para mostrar la información
 *           decodificada de los datos del suelo, posición GNSS, fecha, 
 *           parámetros de calidad de señal (RSSI, SNR) y categoría.
 */
void test_task_lora_rx_should_receive_data(void)
{   
    LoRaReceive_IgnoreAndReturn(DATA_RECEIVED_LEN);
    GetPacketStatus_Ignore();

    logger_log_Expect(LOG_LEVEL_DEBUG, "Soil Data: N:%d P:%d K:%d T:%.2f M:%.2f pH:%.2f C:%d\n");
    logger_log_Expect(LOG_LEVEL_DEBUG, "GNSS Data: Lat:%.6f Lon:%.6f Alt:%.2f\n");
    logger_log_Expect(LOG_LEVEL_DEBUG, "Date: %04d-%02d-%02d %02d:%02d\n");
    logger_log_Expect(LOG_LEVEL_DEBUG, "RSSI: %d dBm, SNR: %d dB\n");
    logger_log_Expect(LOG_LEVEL_DEBUG, "category: %d\n");
    task_lora_rx();
}

/* === End of documentation ==================================================================== */