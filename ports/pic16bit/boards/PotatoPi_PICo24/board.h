/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 ReimuNotMoe of SudoMaker Ltd <reimu@sudomaker.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_PIC16BIT_BOARD_H
#define MICROPY_INCLUDED_PIC16BIT_BOARD_H

#ifndef FCY
#define FCY 32000000
#endif
#include <libpic30.h>

void cpu_init(void);

void led_init(void);
void led_state(int led, int state);
void led_toggle(int led);

void switch_init(void);
int switch_get(int sw);

void uart_init(void);
int uart_rx_any(void);
int uart_rx_char(void);
void uart_tx_char(int chr);

#define MICROPY_HW_BOARD_NAME "PotatoPi PICo24"
#define MICROPY_HW_MCU_NAME "PIC24FJ256GB206"

#endif // MICROPY_INCLUDED_PIC16BIT_BOARD_H
