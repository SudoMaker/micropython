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

#include <xc.h>
#include "board.h"

// CONFIG3
#pragma config WPFP = WPFP255    //Write Protection Flash Page Segment Boundary->Highest Page (same as page 170)
#pragma config SOSCSEL = EC    //Secondary Oscillator Power Mode Select->External clock (SCLKI) or Digital I/O mode(
#pragma config WUTSEL = LEG    //Voltage Regulator Wake-up Time Select->Default regulator start-up time is used
#pragma config ALTPMP = ALPMPDIS    //Alternate PMP Pin Mapping->EPMP pins are in default location mode
#pragma config WPDIS = WPDIS    //Segment Write Protection Disable->Segmented code protection is disabled
#pragma config WPCFG = WPCFGDIS    //Write Protect Configuration Page Select->Last page (at the top of program memory) and Flash Configuration Words are not write-protected
#pragma config WPEND = WPENDMEM    //Segment Write Protection End Page Select->Protected code segment upper boundary is at the last page of program memory; the lower boundary is the code page specified by WPFP

// CONFIG2
#pragma config POSCMOD = XT    //Primary Oscillator Select->XT Oscillator mode is selected
#pragma config IOL1WAY = OFF    //IOLOCK One-Way Set Enable->The IOLOCK bit (OSCCON<6>) can be set once, provided the unlock sequence has been completed. Once set, the Peripheral Pin Select registers cannot be written to a second time.
#pragma config OSCIOFNC = ON    //OSCO Pin Configuration->OSCO/CLKO/RC15 functions as port I/O (RC15)
#pragma config FCKSM = CSECMD    //Clock Switching and Fail-Safe Clock Monitor->Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config FNOSC = FRC    //Initial Oscillator Select->FRC
#pragma config PLL96MHZ = ON    //96MHz PLL Startup Select->96 MHz PLL is enabled automatically on start-up
#pragma config PLLDIV = DIV2    //96 MHz PLL Prescaler Select->Oscillator input is divided by 2 (8 MHz input)
#pragma config IESO = OFF    //Internal External Switchover->IESO mode (Two-Speed Start-up) is disabled

// CONFIG1
#pragma config WDTPS = PS32768    //Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128    //WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = OFF    //Windowed WDT->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF    //Watchdog Timer->Watchdog Timer is disabled
#pragma config ICS = PGx3    //Emulator Pin Placement Select bits->Emulator functions are shared with PGEC3/PGED3
#pragma config GWRP = OFF    //General Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = OFF    //General Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF    //JTAG Port Enable->JTAG port is disabled

/********************************************************************/
// CPU

void cpu_init(void) {
	// CPDIV 1:1; RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled;
	CLKDIV = 0x3100;
	// TUN Center frequency;
	OSCTUN = 0x00;
	// ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled;
	PMD1 = 0x00;
	// OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled;
	PMD2 = 0x00;
	// I2C3MD enabled; PMPMD enabled; U3MD enabled; RTCCMD enabled; CMPMD enabled; CRCMD enabled; I2C2MD enabled;
	PMD3 = 0x00;
	// U4MD enabled; UPWMMD enabled; USB1MD enabled; CTMUMD enabled; REFOMD enabled; LVDMD enabled;
	PMD4 = 0x00;
	// IC9MD enabled; OC9MD enabled;
	PMD5 = 0x00;
	// SPI3MD enabled;
	PMD6 = 0x00;
	// CF no clock failure; NOSC PRIPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete;
	__builtin_write_OSCCONH((uint8_t) (0x03));
	__builtin_write_OSCCONL((uint8_t) (0x01));
	// Wait for Clock switch to occur
	while (OSCCONbits.OSWEN != 0);
	while (OSCCONbits.LOCK != 1);

	__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

	// UART1
	RPINR18bits.U1RXR = 21;
	RPOR13bits.RP26R = 3;

	__builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS

}

/********************************************************************/
// LEDs

#define GREEN_LED_TRIS		TRISFbits.TRISF1
#define BLUE_LED_TRIS		TRISFbits.TRISF0

#define GREEN_LED		LATFbits.LATF1
#define BLUE_LED		LATFbits.LATF0

#define LED_ON (0)
#define LED_OFF (1)

void led_init(void) {
	GREEN_LED_TRIS = 0;
	BLUE_LED_TRIS = 0;

	BLUE_LED = LED_OFF;
	GREEN_LED = LED_OFF;
}

void led_state(int led, int state) {
	switch (led) {
		case 1:
			GREEN_LED = state;
			break;
		case 2:
			BLUE_LED = state;
			break;
	}
}

void led_toggle(int led) {
	switch (led) {
		case 1:
			GREEN_LED ^= 1;
			break;
		case 2:
			BLUE_LED ^= 1;
			break;
	}
}

/********************************************************************/
// switches

#define SWITCH_S1_TRIS		TRISDbits.TRISD7

#define SWITCH_S1		PORTDbits.RD7


void switch_init(void) {
	// set switch GPIO as inputs
	SWITCH_S1_TRIS = 0;
	CNPU2 = 0x0001;
}

int switch_get(int sw) {
	int val = 1;
	switch (sw) {
		case 1:
			val = SWITCH_S1;
			break;
	}
	return val == 0;
}

/********************************************************************/
// UART

/*
// TODO need an irq
void uart_rx_irq(void) {
    if (c == interrupt_char) {
        MP_STATE_VM(mp_pending_exception) = MP_STATE_PORT(keyboard_interrupt_obj);
    }
}
*/

void uart_init(void) {
	// STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX;
	// Data Bits = 8; Parity = None; Stop Bits = 1;
	U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit
	// UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled;
	U1STA = 0x00;
	// BaudRate = 115200; Frequency = 16000000 Hz; BRG 0x22;
	U1BRG = 0x22;

	U1MODEbits.UARTEN = 1;   // enabling UART ON bit
	U1STAbits.UTXEN = 1;
}

int uart_rx_any(void) {
	return U1STAbits.URXDA;
}

int uart_rx_char(void) {
	return U1RXREG;
}

void uart_tx_char(int chr) {
	while (U1STAbits.UTXBF) {
		// tx fifo is full
	}
	U1TXREG = chr;
}
