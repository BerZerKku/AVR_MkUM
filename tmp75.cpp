/*
 * twi.cpp
 *
 *  Created on: 15.01.2016
 *      Author: Shcheblykin
 */

#include <avr/io.h>
#include "tmp75.h"

// Конструктор
TTmp75::TTmp75(uint8_t adr) {

	// fscl = fclk/(16 + 2*TWBR*4^TWPS)
	// При TWPS = 1 и TWBR = 198, получаем: 16000000/(16 + 2*198*4^1) = 10кГц
	TWBR = 198;
	TWSR = (0 << TWPS1) | (1 << TWPS0);
	// Разрешение работы модуля и прерываний
	TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEN);

	this->adrIC = adr << s_u8AdrBits;
	temperature = s_i8TempError;
}

//	Запуск процедуры считывания температуры.
void TTmp75::readTemp() {
	temperature = s_i8TempError;

	uint8_t tmp = TWCR;

	if (((tmp & (1 << TWINT)) == 0) || (temperature == s_i8TempError)) {
		TWCR = (1 << TWINT);
		// формирование СТАРТ
		TWCR = 	(1 << TWEN) | (1 << TWIE)  | (1 << TWINT) | (1 << TWSTA);
	}
}


// Считывание температуры.
int8_t TTmp75::getTemperature() {
	return temperature;
}
