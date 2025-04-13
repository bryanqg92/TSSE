/************************************************************************************************
Copyright (c) 2023, Esteban Volentini <evolentini@herrera.unt.edu.ar>

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

/** @file test_leds.c
 ** @brief Pruebas unitarias para el manejo de los leds
 **/

/* === Headers files inclusions =============================================================== */

#include "unity.h"
#include "leds.h"

/* === Macros definitions ====================================================================== */


/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */
static uint16_t leds_virtuales = ALL_LEDS_ON;
/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */
static uint8_t select_random_led() {
    return (rand() % MAX_LEDS_POSITION) + 1;
}

static uint16_t select_random_multiple_leds() {
    return (rand() % ALL_LEDS_ON) + 1;
}
/* === Public function implementation ========================================================== */

void setUp(){
    LedsInit(&leds_virtuales);
}

//@test Con la inicialización todos los LEDs quedan apagados.
void test_todos_los_leds_inician_apagados (void) {

    uint16_t leds_virtuales = ALL_LEDS_ON;
    LedsInit(&leds_virtuales);
    TEST_ASSERT_EQUAL_HEX16(ALL_LEDS_OFF, leds_virtuales);
}

//@test Prender un LED individual.
void test_prender_led_individual (void) {

    uint8_t led = select_random_led();
    LedsTurnOnSingle(led);
    TEST_ASSERT_EQUAL_HEX16(LedMaskOn(led), leds_virtuales);
}

//@test Apagar un LED individual.
void test_apagar_led_individual (void) {

    uint8_t led = select_random_led();
    LedsTurnOnSingle(led);
    LedsTurnOffSingle(led);
    TEST_ASSERT_EQUAL_HEX16(LedMaskOff(led), leds_virtuales);
}

//@test Prender y apagar múltiples LED’s.
void test_prender_y_apagar_multiples_leds (void) {
    uint16_t leds = select_random_multiple_leds();
    LedsTurnOnMultiple(leds);
    TEST_ASSERT_EQUAL_HEX16(leds, leds_virtuales);
}

//@test Prender todos los LEDs de una vez.
void test_prender_todos_los_leds (void) {
    
    LedsTurnOnAll();
    TEST_ASSERT_EQUAL_HEX16(ALL_LEDS_ON, leds_virtuales);
}

//@test Apagar todos los LEDs de una vez.
void test_apagar_todos_los_leds (void) {
    
    LedsTurnOnAll();
    LedsTurnOffAll();
    TEST_ASSERT_EQUAL_HEX16(ALL_LEDS_OFF, leds_virtuales);
}

//@test Consultar el estado de un LED que está encendido
void test_consultar_estado_led_encendido (void) {
    
    uint8_t led = select_random_led();
    LedsTurnOnSingle(led);
    TEST_ASSERT_TRUE(LedCheckStatus(led));
}

//@test Consultar el estado de un LED que est apagado
void test_consultar_estado_led_apagado (void) {
    
    uint8_t led = select_random_led();
    LedsTurnOnSingle(led);
    LedsTurnOffSingle(led);
    TEST_ASSERT_FALSE(LedCheckStatus(led));
}

//@test Revisar limites de los parametros.
void test_revisar_limites_parametros (void) {
    
    TEST_ASSERT_TRUE(IsLedPositionValid(MIN_LEDS_POSITION));
    TEST_ASSERT_TRUE(IsLedPositionValid(MAX_LEDS_POSITION));
}

//@test Revisar parámetros fuera de los limites.
void test_revisar_parametros_fuera_de_los_limites (void) {
    
    TEST_ASSERT_FALSE(IsLedPositionValid(MIN_LEDS_POSITION - 1));
    TEST_ASSERT_FALSE(IsLedPositionValid(MAX_LEDS_POSITION + 1));
}

/* === End of documentation ==================================================================== */
