/*
 * twi.cpp
 *
 *  Created on: 15.01.2016
 *      Author: Shcheblykin
 */

#include <avr/io.h>
#include "tmp75.h"

// �����������
TTmp75::TTmp75(uint8_t adr) {

	// fscl = fclk/(16 + 2*TWBR*4^TWPS)
	// ��� TWPS = 1 � TWBR = 198, ��������: 16000000/(16 + 2*198*4^1) = 10���
	TWBR = 198;
	TWSR = (0 << TWPS1) | (1 << TWPS0);
	// ���������� ������ ������ � ����������
	TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEN);

	this->adrIC = adr << s_u8AdrBits;
	temperature = s_i8TempError;
}

//	������ ��������� ���������� �����������.
void TTmp75::readTemp() {
	temperature = s_i8TempError;

	uint8_t tmp = TWCR;

	if ((tmp & (1 << TWINT)) == 0) {
		// ������������ �����
		TWCR = 	(1 << TWEN) | (1 << TWIE)  | (1 << TWINT) | (1 << TWSTA);
	}
}


// ���������� �����������.
int8_t TTmp75::getTemperature() {
	return temperature;
}