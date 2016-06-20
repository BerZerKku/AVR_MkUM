/*
 * main.cpp
 *
 *  Created on: 15.06.2016
 *      Author: Shcheblykin
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "tmp75.h"

// DEFINE /////////////////////////////////////////////////////////////////////

/// ���������� ������������ ������� ���
#define NUM_ADC_CHANNEL 6
/// ����� ��� ��������� ������ ��� � �������� ADMUX
#define MUX 0x07
/// ���������� �������� ��� ���������� ������� 2^(AV_LENGH_NUMBER)
#define AV_LENGHT_NUMBER 6
/// ������ ������ UART
#define UART_BUF_LEN 25
/// ���������� ������������ ���� ������
#define UART_TX_LEN 16
/// ����� ������� �����������
#define TEMP_IC_ADR 0x48

// VARIABLE ///////////////////////////////////////////////////////////////////

/// ������ ������� ������������ ������� ���
static const uint8_t aAdcChannel[NUM_ADC_CHANNEL] = { 0, 1, 2, 3, 6, 7 };

/// ������ ����������� �������� ������������ ������� ���
uint16_t aAdcValue[NUM_ADC_CHANNEL] = { 0 };

/// ������� ��������/���������� ���� UART
volatile uint8_t iUartCnt = 0;

/// ���������� ������ �� �������� UART
uint8_t nUartLenTx = 0;

/// ���� ������� ��������� ��������� UART
volatile bool bIsUartRxMessage = false;

/// ����� ������ UART
uint8_t aUartBuf[UART_BUF_LEN] = { 0 };

/// ������ �����������
TTmp75 CTmp75(TEMP_IC_ADR);

// STATIC FUNCTION DECLARATION ////////////////////////////////////////////////

void low_level_init()
		__attribute__((__naked__)) __attribute__((section(".init3")));
static void StartADC();
static void UartTxStart(uint8_t len);
static void UartRxStart();

// FUNCTION DEFINITION ////////////////////////////////////////////////////////

/**	������ ���.
 *
 *	��� ����������� � ������ ������������ ��������������.
 *	�������� ������� ������ = F_CPU / division = 16M / 128 = 125���.
 *	������ ������������� ���������� �� 25 ������ (0.2��), ��������� �� 13 (0.1��).
 *
 */
static void StartADC() {
	ADMUX = (ADMUX & ~MUX) + aAdcChannel[0];
	ADCSRA = (1 << ADEN) |	// enable ADC
			(1 << ADSC) |	// start conversion
			(0 << ADFR) |	// 0 - single conversion / 1 - free runing
			(1 << ADIE) |	// interrupt enable
			(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// division 128
}

/**	�������� ������ �� UART.
 *
 *	���� ���������� ������ ��� �������� ��������� ������ ������ ������ UART,
 *	�������� ������������� �� �����.
 *
 *	@param[in] len ���������� ������ ��� ��������.
 */
static void UartTxStart(uint8_t len) {
	if ((len > 0) && (len <= UART_BUF_LEN)) {
		iUartCnt = 0;
		nUartLenTx = len;
		UCSRB |= (1 << TXEN) | (1 << UDRIE) | (1 << TXCIE);
	}
}

/**	����� ������ ��������� UART
 *
 *	���������� �������� UART � ��������� �� ������.
 *	������� �������� ���� ����������.
 */
static void UartRxStart() {
	UCSRB |= (1 << RXEN) | (1 << RXCIE);
	iUartCnt = 0;
}

/**	��������� �������� ������ �� UART.
 *
 *	@param[in] state ��������� ��������� 0 - ������ ���, ����� ����.
 *	@param[in] byte �������� ���� ������.
 */
static void UartProtocol(uint8_t state, uint8_t byte) {
	static uint8_t step = 0;
	static uint8_t len = 0;

	PORTB ^= (1 << PB1);

	if ((state) || (step >= UART_BUF_LEN)) {
		step = 0;
	} else {

		aUartBuf[step] = byte;
		switch(step) {
			case 0:
				if (byte == 0x55)
					step++;

				break;
			case 1:
				if (byte == 0xAA)
					step++;
				break;

			case 2:
				step++;
				break;

			case 3:
				len = byte + 4;
				step = (len >= UART_BUF_LEN) ? 0 : step + 1;
				break;

			default:
				if (step < len) {
					step++;
				} else {

					uint8_t crc = 0;
					for(uint_fast8_t i=2; i < (len - 2); i++) {
						crc += aUartBuf[i];
					}
					if (aUartBuf[len - 1] == crc) {
						bIsUartRxMessage = true;
						UCSRB &= ~((1 << RXEN) | (1 << RXCIE));
					}
					step = 0;
				}
		}
	}
}

/**	���������� ��������� � ��������.
 *
 * 	����������� ������-����� �
 */

int __attribute__ ((OS_main)) main() {

	sei();

	StartADC();
	UartRxStart();

	while (1) {
		if (bIsUartRxMessage) {
			uint8_t len = 0;
			switch(aUartBuf[2]) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x03: {
					aUartBuf[0] = 0x55;
					aUartBuf[1] = 0xAA;
					aUartBuf[2] = 0x03;
					aUartBuf[3] = 0;		// ���-�� ���� ������ ����� ������������ �����
					len = 4;
					for(uint_fast8_t i = 0; i < NUM_ADC_CHANNEL; i++) {
						uint_fast16_t val =  aAdcValue[i] >> AV_LENGHT_NUMBER;
						aUartBuf[len++] = val;
						aUartBuf[len++] = val >> 8;
					}
					aUartBuf[len++] = CTmp75.getTemperature();
					aUartBuf[len++] = 0x00;
					aUartBuf[3] = len  - 4;	// ���-�� ���� ������
					uint8_t crc = 0;
					for(uint_fast8_t i = 0; i < (aUartBuf[3] + 2); i++) {
						crc += aUartBuf[2 + i];
					}
					aUartBuf[len++] = crc;
					UartTxStart(len);
				} break;
			}

			if (len > 0) {
				UartTxStart(len);
			} else {
				UartRxStart();
			}

			bIsUartRxMessage = false;
		}

		CTmp75.readTemp();
		// ���������� ������ �������� �� ����� 5��, ����� �� ����� ��������
		// ��������� ����� ������� �����������
		_delay_ms(10);
	}
}

/**	��������� �� ����������� ������ �������� UART.
 *
 *	�������� ��������� ���� � ����� ����������� UART. ���� ������ �� �������
 *	������ ���, ���������� �� ����������� �����������.
 */
ISR(USART_UDRE_vect) {
	if (iUartCnt < nUartLenTx) {
		UDR = aUartBuf[iUartCnt++];
	} else {
		UCSRB &= ~(1 << UDRIE);
	}
}

/**	���������� �� ��������� �������� UART.
 *
 *	����� �������� ���������� ����� ������ ����������� UART ���������������.
 *	���������� �� ��������� �������� ���� �����������.
 *
 *	���������� �������� UART � ��������� �� ����. ������� ������ ����������.
 *
 */
ISR(USART_TXC_vect) {
	UCSRB &= ~((1 << TXEN) | (1 << TXCIE));
	UartRxStart();
}

/**	���������� �� ��������� ������ UART.
 *
 */
ISR(USART_RXC_vect) {
	uint8_t state = UCSRA;
	uint8_t byte = UDR;

	UartProtocol(state & ((1 << FE) | (1 << DOR) | (1 << PE)), byte);
}

/**	���������� ���.
 *
 *	��� ��������� �������� ��� ������� ����������� ADCL, � ����� ADCH.
 *
 *	���������� ������� ������� T = K / SPS
 *	, ��� ���������� ����������� ������� � =(2 ^ \a #AV_LENGHT_NUMBER)
 *	SPS - ������� ������������� �������.
 *
 *	��������, �������� ������� 16���, �������� ��� 128, �������������� 13 ������.
 *	�������� SPS = 16M / 128 / 13 ~ 9600. K = 2^6 = 64.
 *	T = 64/9600 = 6.6��.
 */
ISR(ADC_vect) {
	static uint8_t iChannel = 0;
	uint16_t acc = aAdcValue[iChannel];
	uint16_t val = ADC;

	// ����������/����������
	acc = acc - (acc >> AV_LENGHT_NUMBER) + val;
	aAdcValue[iChannel] = acc;

	iChannel = (iChannel + 1) % NUM_ADC_CHANNEL;
	// ����� ���������� ������ � ������ ��������������
	ADMUX = (ADMUX & ~MUX) + aAdcChannel[iChannel];
	ADCSRA |= (1 << ADSC);
}

/**	���������� TWI.
 *
 */
 ISR(TWI_vect) {
	uint8_t state = TWSR;

	CTmp75.isr(state);
}

/**	��������� ������������� ���������.
 *
 *	�� ����������� �� �������� ������ 16 ���.
 *
 *
 * 	������ 0 ����������� ������ 3.2 ��.
 * 	������ 1 ����������� ������ 125 ��.
 */
void low_level_init() {
	// PORTB
	// PB.0 DEBUG TP1
	// PB.1 DEBUG TP2
	// PB.6			XTAL1
	// PB.7			XTAL2
	DDRB = (1 << PB1) | (1 << PB0);
	PORTB = 0x00;

	// PORTC
	// PC.0	alt_in 		ADC0
	// PC.1	alt_in 		ADC1
	// PC.2 alt_in 		ADC2
	// PC.3 alt_in 		ADC3
	// PC.4	alt_bi_hi 	SDA
	// PC.5	alt_out_hi 	SCL
	// PC.6 alt_in		ADC6
	// PC.7	alt_in		ADC7
	DDRC = 0x00;
	PORTC = (1 << PC4) | (1 << PC5);

	// PORTD
	// PD.0	alt_in	RXD
	// PD.1	alt_out	TXD
	DDRD = (1 << PD1);
	PORTD = (1 << PD1);

	// UART
	// UBRR = 16M/(16*1200) - 1 = 832 (U2X = 0)
	UCSRA = (0 << U2X);
	UCSRB = 0x00;
	UCSRC = (1 << URSEL) |								// write to UCSRC
			(0 << UPM1)  | (0 << UPM0) |				// parity mode disabled
			(1 << USBS)  |								// 2 stop bits
			(0 << UCSZ2) | (1 << UCSZ1) | (1 << UCSZ0);	// 8-bit character size
	static const uint16_t ubrr = (F_CPU / 16) / 1200 - 1;
	UBRRH = (uint8_t) (ubrr >> 8); //
	UBRRL = (uint8_t) ubrr; //

	// ADC
	ADMUX = (0 << REFS1) | (1 << REFS0) |	// AVcc with cap at REF
//			(1 << REFS1) | (1 << REFS0) | 	// internal 2.56V with cap at AREF
			(0 << ADLAR);					// right adjust result

	// TIMER0

	// TWI
}
;
