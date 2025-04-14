/************************************************************************************************
Copyright (c) 2025 Leandro Quiroga bryanqg92@gmail.com

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

#ifndef LEDS_H
#define LEDS_H

/** @file
 ** @brief Declaración de la biblioteca para control de leds
 **/

/* === Headers files inclusions ================================================================ */

/* === Headers files inclusions ================================================================ */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/* === Cabecera C++ ============================================================================ */

#ifdef __cplusplus
/**
 * @brief Inicializa el sistema de control de LEDs.
 *
 * Esta función configura el sistema de LEDs, inicializando el estado de los mismos
 * y preparando el hardware necesario para su control. Debe ser llamada antes de 
 * realizar cualquier operación con los LEDs.
 *
 * @param leds Máscara de bits que representa los LEDs a inicializar. Cada bit 
 *             corresponde a un LED específico, donde un valor de 1 indica que 
 *             el LED está habilitado y un valor de 0 indica que está deshabilitado.
 */
extern "C" {
#endif

/* === Public macros definitions =============================================================== */
/**
 * @def ALL_LEDS_ON
 * @brief Macro para encender todos los LEDs. 
 *        Establece todos los bits en el registro de LEDs a 1.
 */
#define ALL_LEDS_ON 0xFFFF

/**
 * @def ALL_LEDS_OFF
 * @brief Macro para apagar todos los LEDs. 
 *        Establece todos los bits en el registro de LEDs a 0.
 */
#define ALL_LEDS_OFF 0x0000

/**
 * @def MIN_LEDS_POSITION
 * @brief Posición mínima válida para un LED. 
 *        Representa el primer LED en la secuencia.
 */
#define MIN_LEDS_POSITION 1

/**
 * @def MAX_LEDS_POSITION
 * @brief Posición máxima válida para un LED. 
 *        Representa el último LED en la secuencia.
 */
#define MAX_LEDS_POSITION 16


/* === Public data type declarations =========================================================== */

/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

/* Comentario normal */
/**
 * @brief Genera una máscara para encender un LED específico.
 *
 * @param led Número del LED a encender (1 a 16).
 * @return uint16_t Máscara correspondiente al LED especificado.
 */
uint16_t LedMaskOn(uint8_t led);

/**
 * @brief Genera una máscara para apagar un LED específico.
 *
 * @param led Número del LED a apagar (1 a 16).
 * @return uint16_t Máscara correspondiente al LED especificado.
 */
uint16_t LedMaskOff(uint8_t led);

/**
 * @brief Inicializa el sistema de LEDs.
 *
 * @param dirección Puntero a la dirección de memoria donde se almacenará el estado de los LEDs.
 */
void LedsInit(uint16_t *dirección);

/**
 * @brief Enciende un LED específico.
 *
 * @param led Número del LED a encender (1 a 16).
 */
void LedsTurnOnSingle(uint8_t led);

/**
 * @brief Apaga un LED específico.
 *
 * @param led Número del LED a apagar (1 a 16).
 */
void LedsTurnOffSingle(uint8_t led);

/**
 * @brief Enciende múltiples LEDs especificados por una máscara.
 *
 * @param leds Máscara que indica los LEDs a encender.
 */
void LedsTurnOnMultiple(uint16_t leds);

/**
 * @brief Enciende todos los LEDs.
 */
void LedsTurnOnAll(void);

/**
 * @brief Apaga todos los LEDs.
 */
void LedsTurnOffAll(void);

/**
 * @brief Verifica el estado de un LED específico.
 *
 * @param led Número del LED a verificar (1 a 16).
 * @return true Si el LED está encendido.
 * @return false Si el LED está apagado.
 */
bool LedCheckStatus(uint8_t led);

/**
 * @brief Verifica si la posición de un LED es válida.
 *
 * @param led Número del LED a verificar (1 a 16).
 * @return true Si la posición es válida.
 * @return false Si la posición no es válida.
 */
bool IsLedPositionValid(uint8_t led);



#ifdef __cplusplus
}
#endif

#endif // LEDS_H