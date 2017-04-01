/* 4-channels 12V LED dimmer project
 *
 *    Copyright (C) 2017  Vasim V.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program. */

#include "stm32f0xx_hal.h"
// #include "cmsis/device/stm32f030x6.h"
#include "dimmer.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

// Data calculated from ADC readings
// Supply voltage
volatile int32_t Vdd = 0;
// Voltage of main supply power
volatile int32_t Vin = 0;
// Voltage of PHR1
volatile int32_t Vphr1 = 0;
// Voltage of PHR2
volatile int32_t Vphr2 = 0;
// Voltage of ADC3
volatile int32_t Vadc3 = 0;
// Voltage of ADC5
volatile int32_t Vadc5 = 0;
// Chip's temperature
volatile int32_t Tchip = 0;

uint8_t MyModbusAddress;

// Located in flash, will be read into MyModbusAddress at the start
const FlashMyModbusAddress = MODBUS_ADDRESS;

// Channels value array
uint16_t  Pwm[MY_PWM_CHANNELS] = {0, 0, 0, 0};

// ADC DMA buffer
volatile uint16_t AdcData[MY_ADC_CHANNELS];

// ADC sum buffer and counter
volatile uint64_t AdcSumData[MY_ADC_CHANNELS];
volatile uint32_t AdcCount = 0;

// Calculated averages for all ADCs
uint16_t AdcAvgData[MY_ADC_CHANNELS];

// UART Receive buffer (just one character actually)
volatile char RecvBuf[1];

// Modbus frame receive buffers (switches while processing)
uint8_t bufModbus[2][MODBUS_FRAME_BUFFER];
uint8_t curBuf = 0;
uint16_t cntBufRecv = 0;

// Address of frame and its length to process (NULL if nothing)
uint8_t *ModbusFrame = NULL;
uint16_t cntModbus = 0;

// Output modbus buffer
uint8_t bufModbusOut[64];
uint16_t cntOut = 0;

void PWM_Timer_Set(TIM_HandleTypeDef *phtim, uint32_t channel, uint32_t pulse, uint8_t inverted);

volatile uint8_t txDoneFlag = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    txDoneFlag = 1;
} // void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

// Send bytes array to the serial port
void SerialPut(uint8_t *Buf, int Len) {
    while (!txDoneFlag);
    txDoneFlag = 0;
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
    HAL_UART_Transmit_DMA(&huart1, Buf, Len);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
} // void SerialPut(uint8_t *Buf, int Len)

#ifdef SERIAL_DEBUG
va_list args;
char seroutbuf[128];

void SerialPrint(char *fmt, ...) {
	int nlen;

    va_start(args, fmt);
    nlen = vsnprintf(seroutbuf, sizeof(seroutbuf), fmt, args);
    va_end(args);
    SerialPut((uint8_t *) seroutbuf, nlen);
} // void SerialPrint(char *fmt, ...)
#endif

// UART receiver callback, puts received char in CmdBuf (and to CmdNext if available)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	int len;

	// Check idle time for modbus
	if (__HAL_TIM_GET_COUNTER(&htim16) > MODBUS_FRAME_IDLE) {
		if (cntBufRecv > 0) {
			// Switch buffers and signal to main thread
			ModbusFrame = bufModbus[curBuf];
			cntModbus = cntBufRecv;
			curBuf = curBuf ? 0 : 1;
			// Start new frame reception
			cntBufRecv = 0;
		}
	}
	// Reset idle time for modbus
	__HAL_TIM_SET_COUNTER(&htim16, 0);
#ifdef SERIAL_DEBUG
	// Echo commands when debugging
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) RecvBuf, 1);
#endif
	bufModbus[curBuf][cntBufRecv] = RecvBuf[0];
	cntBufRecv++;

	// Garbage in the receive buffer, discarding
	if (cntBufRecv >= MODBUS_FRAME_BUFFER)
		cntBufRecv = 0;
} // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

// Calculate CRC for modbus frame
uint16_t ModbusCrc(uint8_t *buf, int len) {
	uint32_t tmp, tmp2;
	uint8_t Flag;
    uint16_t i, j;

    tmp = 0xFFFF;
    for (i = 0; i < len; i++) {
        tmp = tmp ^ buf[i];
        for (j = 1; j <= 8; j++) {
            Flag = tmp & 0x0001;
            tmp >>=1;
            if (Flag)
                tmp ^= 0xA001;
        }
    }
    tmp2 = tmp >> 8;
    tmp = (tmp << 8) | tmp2;
    tmp &= 0xFFFF;
    return (uint16_t) tmp;
} // uint16_t ModbusCrc(char *buf, int len)

// Add CRC to output modbus packet
void AddModbusCrc() {
	uint16_t tCrc;

	tCrc = ModbusCrc(bufModbusOut, cntOut);
	bufModbusOut[cntOut] = (tCrc >> 8) & 0xff;
	cntOut++;
	bufModbusOut[cntOut] = tCrc & 0xff;
	cntOut++;
} // void AddModbusCrc()

// Write a variable to the flash (can write only 1 bits to 0, be careful)
void FlashVariable(uint16_t *Addr, uint16_t Val) {
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Addr, Val);
    HAL_FLASH_Lock();
} // void FlashVariable(uint16_t *Addr, uint16_t Val)

// Set output holding register (from modbus frame)
void SetRegister(uint16_t Addr, uint16_t Val) {
	switch(Addr) {
	case 0:
		if (Pwm[0] == Val)
			return;
		Pwm[0] = Val;
		PWM_Timer_Set(&htim3, TIM_CHANNEL_1, Val, 0);
		break;
	case 1:
		if (Pwm[1] == Val)
			return;
		Pwm[1] = Val;
		PWM_Timer_Set(&htim14, TIM_CHANNEL_1, Val, 0);
		break;
	case 2:
		if (Pwm[2] == Val)
			return;
		Pwm[2] = Val;
		PWM_Timer_Set(&htim3, TIM_CHANNEL_4, Val, 1);
		break;
	case 3:
		if (Pwm[3] == Val)
			return;
		Pwm[3] = Val;
		PWM_Timer_Set(&htim3, TIM_CHANNEL_2, Val, 1);
		break;
	case 256:
		// Set my modbus address (it is one-time thing, you will have to erase FLASH to reset!!!
		MyModbusAddress = Val;
		FlashVariable(&FlashMyModbusAddress, Val);
		break;
	default:
		break;
	}
} // void SetRegister(uint16_t Addr, uint16_t Val)

// Get input register (for modbus master)
uint16_t GetRegister(uint16_t Addr) {
	switch(Addr) {
	case 0:
		return Vin;
	case 1:
		return Vdd;
	case 2:
		return Tchip;
	case 3:
		return Vphr1;
	case 4:
		return Vphr2;
	case 5:
		return Vadc3;
	case 6:
		return Vadc5;
	default:
		break;
	}
	return 0;
} // uint16_t GetRegister(uint16_t Addr)

// Check and process modbus frame, create reply in
int ProcessModbusFrame(uint8_t *Frame, uint16_t len) {
	int i;
	int sum;
	uint16_t rCrc;
	uint16_t tAddr, tLen, tVal, tCnt;

	// Not my address
	if (Frame[0] != MyModbusAddress)
		return;

	// Check for bad CRC
	rCrc = (Frame[cntModbus - 2] << 8) | Frame[cntModbus - 1];
	if (rCrc != ModbusCrc(Frame, len - 2)) {
#ifdef SERIAL_DEBUG
		HAL_Delay(2);
		SerialPrint("bad CRC %x %x\n",
				rCrc, ModbusCrc(Frame, len - 2));
		HAL_Delay(2);
#endif
		return;
	}
	// Preparing header for output
	bufModbusOut[0] = MyModbusAddress;
	bufModbusOut[1] = Frame[1];
	cntOut = 2;

	// Checking incoming message
	tAddr = (Frame[2] << 8) | Frame[3];
	switch (Frame[1]) {
	// Read holding registers (output)
	case 0x03:
		tLen = (Frame[4] << 8) | Frame[5];
		tCnt = 0;
		cntOut++;
		for (i = tAddr; i < (tAddr + tLen); i++) {
			// Fill output buffer with values
			if ((i >= 0) && (i <= 3)) {
				tVal = Pwm[i];
			} else
				tVal = 0;
			bufModbusOut[cntOut] = (uint8_t) ((tVal >> 8) & 0xff);
			cntOut++;
			bufModbusOut[cntOut] = (uint8_t) (tVal & 0xff);
			cntOut++;
			tCnt++;
			// Break if reply is too long
			if (cntOut > (sizeof(bufModbusOut) - 8))
				break;
		}
		bufModbusOut[2] = tCnt * 2;
		break;
	// Read input registers (ADC inputs)
	case 0x04:
		tLen = (Frame[4] << 8) | Frame[5];
		tCnt = 0;
		cntOut++;
		for (i = tAddr; i < (tAddr + tLen); i++) {
			tVal = GetRegister(i);
			bufModbusOut[cntOut] = (uint8_t) ((tVal >> 8) & 0xff);
			cntOut++;
			bufModbusOut[cntOut] = (uint8_t) (tVal & 0xff);
			cntOut++;
			tCnt++;
			// Break if reply is too long
			if (cntOut > (sizeof(bufModbusOut) - 8))
				break;
		}
		bufModbusOut[2] = tCnt * 2;
		break;
	// Write single holding register (output)
	case 0x06:
		tVal = (Frame[4] << 8) | Frame[5];

		SetRegister(tAddr, tVal);
		bufModbusOut[cntOut] = Frame[2];
		cntOut++;
		bufModbusOut[cntOut] = Frame[3];
		cntOut++;
		bufModbusOut[cntOut] = Frame[4];
		cntOut++;
		bufModbusOut[cntOut] = Frame[5];
		cntOut++;
		break;
	// Write multiple holding registers (output)
	case 0x10:
		tLen = (Frame[4] << 8) | Frame[5];
		for (i = tAddr; i < (tAddr + tLen); i ++) {
			tVal = (Frame[(i - tAddr) * 2 + 7] << 8) | Frame[(i - tAddr) * 2 + 8];
			SetRegister(i, tVal);
		}
		bufModbusOut[cntOut] = Frame[2];
		cntOut++;
		bufModbusOut[cntOut] = Frame[3];
		cntOut++;
		bufModbusOut[cntOut] = Frame[4];
		cntOut++;
		bufModbusOut[cntOut] = Frame[5];
		cntOut++;
		break;
	// Unknown code, sending bad function reply
	default:
		bufModbusOut[1] = 0x80 | Frame[1];
		bufModbusOut[2] = 0x01;
		cntOut = 3;
		return;
	}
	AddModbusCrc();
} // void ProcessModbusFrame()

// Read values from ADC buffer and calculate voltages
void UpdateThings() {
	// Calculate averages for readings
	for (int i = 0; i < MY_ADC_CHANNELS; i ++) {
		AdcAvgData[i] = AdcSumData[i] / AdcCount;
		AdcSumData[i] = 0;
	}
	AdcCount = 0;
	// Calculate Vdd based on VREFINT_CAL, then use it to calculate other stuff
	Vdd = VREF_CORRECTION * VREFINT_CAL / AdcAvgData[6];
	Vin = (Vdd * AdcAvgData[0]) / (uint16_t) (4095 / DIVIDER_RATIO);
	Vphr1 = (Vdd * AdcAvgData[1]) / (uint16_t) 4095;
	Vphr2 = (Vdd * AdcAvgData[2]) / (uint16_t) 4095;
	Vadc3 = (Vdd * AdcAvgData[3]) / (uint16_t) (4095 / DIVIDER_RATIO);
	Vadc5 = (Vdd * AdcAvgData[4]) / (uint16_t) (4095 / DIVIDER_RATIO);
	Tchip = (((int32_t) AdcAvgData[5]) *  Vdd / (int32_t) 3300) - (int32_t) TS_CAL1;
	Tchip *= (int32_t)(11000 - 3000);
	Tchip = Tchip / (int32_t)( TS_CAL2 - TS_CAL1);
	Tchip += 3000;
//#ifdef SERIAL_DEBUG
//	SerialPrint("A0: %d, A1: %d, A2: %d, A3: %d, A4: %d, A5: %d, A6: %d\n",
//				AdcData[0], AdcData[1], AdcData[2], AdcData[3], AdcData[4], AdcData[5], AdcData[6]);
//	SerialPrint("Cnt: %u, Vdd: %u, Tchip: %u, Vin: %u\n", AdcCount, Vdd, Tchip, Vin);
//#endif
} // void UpdateThings()

// Start 12-bit ADC (5 ADC channels, temperature, vrefint)
void StartAdc() {
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)AdcData, MY_ADC_CHANNELS);
} // void StartAdc()

// DMA completion routine (sum all ADCs readings for averaging later)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	AdcCount++;
	for (int i = 0; i < MY_ADC_CHANNELS; i++) {
		AdcSumData[i] += AdcData[i];
	}
	StartAdc();
} // void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)

// Start debug serial interface
void StartSerial() {
#ifdef SERIAL_DEBUG
	SerialPrint("D!\n");
#endif
	HAL_UART_Receive_DMA(&huart1, (uint8_t *) RecvBuf, sizeof(RecvBuf));
} // void StartSerial()

// Set up duty cycle on a timer
void PWM_Timer_Set(TIM_HandleTypeDef *phtim, uint32_t channel, uint32_t pulse, uint8_t inverted) {
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	if (inverted) {
		sConfigOC.Pulse = PWMRANGE - pulse + 1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	} else {
		sConfigOC.Pulse = pulse;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	}
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_Stop(phtim, channel);
	HAL_TIM_PWM_ConfigChannel(phtim, &sConfigOC, channel);
	HAL_TIM_PWM_Start(phtim, channel);
} // PWM_Timer_Set

// Start timers
void StartTimers() {
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_Base_Start(&htim16);
} // void StartTimers()

// Start watchdog
void StartIwdg() {
	HAL_IWDG_Start(&hiwdg);
} // void StartIwdg()

// Setup things before run
void Setup() {
#ifdef SERIAL_DEBUG
	DBGMCU->CR = 0x7;
#endif
	MyModbusAddress = FlashMyModbusAddress;
	StartSerial();
	StartTimers();
	StartAdc();
	// StartIwdg();
} // void Setup()

uint64_t LastUpdate = -9000;

// Main program loop
void Loop() {
	if ((HAL_GetTick() - LastUpdate) > REPORT_PERIOD) {
		UpdateThings();
		LastUpdate = HAL_GetTick();
#ifdef SERIAL_DEBUG
 	SerialPrint("0:%d\n1:%d\n2:%d\n3:%d\n4:%d\n5:%d\n6:%d\n",
				Vin, Vphr1, Vphr2, Vadc3, Vadc5, Vdd, Tchip);
#endif
	}
	// Check if there is new modbus frame waiting for us
	if (__HAL_TIM_GET_COUNTER(&htim16) > MODBUS_FRAME_IDLE) {
		if (cntBufRecv > 0) {
			// Switch buffers and signal to main thread
			ModbusFrame = bufModbus[curBuf];
			cntModbus = cntBufRecv;
			curBuf = curBuf ? 0 : 1;
			// Start new frame reception
			cntBufRecv = 0;
		} else
			HAL_UART_Receive_DMA(&huart1, (uint8_t *) RecvBuf, sizeof(RecvBuf));
	}
	// Process modbus frame if we received it
	if (ModbusFrame) {
#ifdef SERIAL_DEBUG
		SerialPrint("F %d, %d, %d)\n", ModbusFrame[0], ModbusFrame[1], cntModbus);
#endif
		ProcessModbusFrame(ModbusFrame, cntModbus);
		ModbusFrame = NULL;
		cntModbus = 0;
		__HAL_TIM_SET_COUNTER(&htim16, 0);
	}
	// Make sure there is delay before sending reply)
	if ((cntOut > 0) && (__HAL_TIM_GET_COUNTER(&htim16) > MODBUS_FRAME_IDLE)) {
#ifdef SERIAL_DEBUG
		SerialPrint("R %d\n", cntOut);
#endif
		SerialPut(bufModbusOut, cntOut);
		cntOut = 0;
	}
	// HAL_IWDG_Refresh(&hiwdg);
} // void loop()
