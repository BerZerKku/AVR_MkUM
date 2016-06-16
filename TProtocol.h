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

	/** ��������� ������ ���������.
	 *
	 * 	\attention
	 * 	#STATE_OFF - ������ ������ ������ � ������ ������������.
	 *
	 * 	\attention
	 * 	#STATE_ERROR - ������ ������ ������ � ����� ������������.
	 *
	 */
	typedef enum __attribute__ ((__packed__)) {
		STATE_OFF 			= 0, ///< \b <0> �������� ��������.
		// ^^^ - ������ �������
		STATE_READ			= 1, ///< \b <1> ���� ������ �������.
		STATE_READ_ERROR	= 2, ///< \b <2> ������ � �������� ������.
		STATE_READ_OK		= 3, ///< \b <3> ������� �������.
		STATE_WRITE_WAIT	= 4, ///< \b <4> �������� ������ ������.
		STATE_WRITE_READY	= 5, ///< \b <5> ������� ������ � ��������
		STATE_WRITE			= 6, ///< \b <6> ���� �������� �������.
		// vvv - ������ � �����
		STATE_ERROR			= 7  ///< \b <7> ������ � ������ ���������.
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
