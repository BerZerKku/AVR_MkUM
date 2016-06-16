/*
 * TProtocol.h
 *
 *  Created on: 16.06.2016
 *      Author: Shcheblykin
 */

#ifndef TPROTOCOL_H_
#define TPROTOCOL_H_

#include "stdio.h"

class TProtocol {
public:

	/** Состояния работы протокола.
	 *
	 * 	\attention
	 * 	#STATE_OFF - всегда должен стоять в начале перечисления.
	 *
	 * 	\attention
	 * 	#STATE_ERROR - всегда должен стоять в конце перечисления.
	 *
	 */
	typedef enum __attribute__ ((__packed__)) {
		STATE_OFF 			= 0, ///< \b <0> Протокол выключен.
		// ^^^ - всегда вначале
		STATE_READ			= 1, ///< \b <1> Идет чтение посылки.
		STATE_READ_ERROR	= 2, ///< \b <2> Ошибка в принятом пакете.
		STATE_READ_OK		= 3, ///< \b <3> Посылка принята.
		STATE_WRITE_WAIT	= 4, ///< \b <4> Ожидание нужных данных.
		STATE_WRITE_READY	= 5, ///< \b <5> Посылка готова к отправке
		STATE_WRITE			= 6, ///< \b <6> Идет отправка посылки.
		// vvv - всегда в конце
		STATE_ERROR			= 7  ///< \b <7> Ошибка в работе протокола.
	} TState;

	TProtocol();

	void SetEnable() {

	}

	void SetDisable() {

	}

	bool isEnable() const {

	}

	void setReadState() {

	}

	bool isReadData() const {

	}

	void setState() {

	}

	bool checkState() {

	}

	uint8_t push(uint8_t byte);

	uint8_t getNumOfBytes() const {

	}

	uint16_t setTick(uint16_t baudrate, uint8_t period);

	void tick();

	bool setAddressLan(uint8_t adr);

	uint8_t getAddressLan() const {

	}

	uint8_t sendData();

	TError readData();
};

#endif /* TPROTOCOL_H_ */
