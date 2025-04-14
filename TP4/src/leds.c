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

/** @file leds.c
 ** @brief 
 **/

/* === Headers files inclusions =============================================================== */
#include "leds.h"

/* === Macros definitions ====================================================================== */
/**
 * @def LED_BIT_ON
 * @brief Macro que representa el valor del bit para un LED en estado ENCENDIDO.
 */
#define LED_BIT_ON 1

/**
 * @def LED_BIT_OFF
 * @brief Macro que representa el valor del bit para un LED en estado APAGADO.
 */
#define LED_BIT_OFF 0

/* === Private data type declarations ========================================================== */
static uint16_t *port_adress;
/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */


/* === Public function implementation ========================================================== */
/**
 * @brief Verifica si la posición del LED es válida.
 *
 * Esta función comprueba si la posición especificada del LED está dentro
 * del rango válido definido por MIN_LEDS_POSITION y MAX_LEDS_POSITION.
 *
 * @param led La posición del LED a validar (índice basado en 1).
 * @return true si la posición del LED es válida, false en caso contrario.
 */
bool IsLedPositionValid(uint8_t led) {
    return (led >= MIN_LEDS_POSITION && led <= MAX_LEDS_POSITION);
}

/**
 * @brief Genera la máscara para encender un LED específico.
 *
 * Esta función calcula la máscara necesaria para encender un LED en la posición
 * especificada, desplazando el bit correspondiente.
 *
 * @param led La posición del LED (índice basado en 1).
 * @return La máscara para encender el LED.
 */
uint16_t LedMaskOn(uint8_t led) {
    return LED_BIT_ON << (led - 1);
}

/**
 * @brief Inicializa el controlador de LEDs.
 *
 * Esta función configura la dirección del puerto que controla los LEDs y
 * apaga todos los LEDs al inicio.
 *
 * @param dirección Puntero a la dirección del puerto de control de LEDs.
 */
void LedsInit(uint16_t *dirección) {
    port_adress = dirección;
    *dirección = ALL_LEDS_OFF;
}

/**
 * @brief Enciende un LED individual.
 * 
 * Esta función enciende el LED especificado por el parámetro `led`.
 * Si la posición del LED no es válida, la función retorna sin realizar cambios.
 * 
 * @param led La posición del LED a encender (índice basado en 1).
 */
void LedsTurnOnSingle(uint8_t led) {
    if (!IsLedPositionValid(led)) {
        return;
    }
    *port_adress |= LedMaskOn(led);
}

/**
 * @brief Apaga un LED individual.
 * 
 * Esta función apaga el LED especificado por el parámetro `led`.
 * Si la posición del LED no es válida, la función retorna sin realizar cambios.
 * 
 * @param led La posición del LED a apagar (índice basado en 1).
 */
void LedsTurnOffSingle(uint8_t led) {
    if (!IsLedPositionValid(led)) {
        return;
    }
    *port_adress &= ~(LedMaskOn(led));
}

/**
 * @brief Enciende todos los LEDs.
 * 
 * Esta función enciende todos los LEDs configurando el puerto de control
 * con la máscara que representa todos los LEDs encendidos.
 */
void LedsTurnOnAll(void) {
    *port_adress = ALL_LEDS_ON;
}

/**
 * @brief Apaga todos los LEDs.
 * 
 * Esta función apaga todos los LEDs configurando el puerto de control
 * con la máscara que representa todos los LEDs apagados.
 */
void LedsTurnOffAll(void) {
    *port_adress = ALL_LEDS_OFF;
}

/**
 * @brief Verifica el estado de un LED.
 * 
 * Esta función verifica si el LED especificado por el parámetro `led`
 * está encendido o apagado.
 * 
 * @param led La posición del LED a verificar (índice basado en 1).
 * @return true si el LED está encendido, false si está apagado o si la posición no es válida.
 */
bool LedCheckStatus(uint8_t led) {
    if (!IsLedPositionValid(led)) {
        return false;
    }
    return (*port_adress & LedMaskOn(led)) != LED_BIT_OFF;
}




/* === End of documentation ==================================================================== */
