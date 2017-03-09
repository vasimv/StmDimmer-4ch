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
char RecvBuf[1];

// Command to process
volatile char CmdNext[MAX_COMMAND_LENGTH];
// We have a command in the buffer, waiting to process it
volatile uint8_t FlagProcessCmd = 0;

// Buffer for commands waiting for processing
char CmdBuf[256];
int CmdBufLength = 0;

char seroutbuf[256];

va_list args;
int nlen;
volatile uint8_t txDoneFlag = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    txDoneFlag = 1;
} // void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

void SerialPrint(char *fmt, ...) {
    va_start(args, fmt);
    nlen = vsnprintf(seroutbuf, sizeof(seroutbuf), fmt, args);
    va_end(args);
    while (!txDoneFlag);
    txDoneFlag = 0;
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *) seroutbuf, nlen);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) && (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX));
} // void SerialPrint(char *fmt, ...)

// UART receiver callback, puts received char in CmdBuf (and to CmdNext if available)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	char *pNext;
#ifdef SERIAL_DEBUG
	// Echo commands when debugging
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) RecvBuf, 1);
#endif
	// Check if we have enough space in buffer for new chars
	if (CmdBufLength < sizeof(CmdBuf)) {
		CmdBuf[CmdBufLength] = *RecvBuf;
		CmdBufLength++;
	}
	// Check if we can process and have new commands waiting in CmdBuf
	if (FlagProcessCmd)
		return;
	pNext = memchr(CmdBuf, '\n', CmdBufLength);
	if (pNext) {
		// Found EOL char in the buffer, moving command to CmdNext processing buffer
		int len = pNext - CmdBuf + 1;

		*pNext = '\0';
		if (len < (sizeof(CmdNext) - 1)) {
			memcpy(CmdNext, CmdBuf, sizeof(CmdNext) - 1);
			CmdNext[sizeof(CmdNext) - 1] = '\0';
		} else
			memcpy(CmdNext, CmdBuf, len);
		FlagProcessCmd = 1;
		// Remove the command from CmdBuf
		if (len < CmdBufLength)
		memcpy(CmdBuf, pNext + 1, CmdBufLength - len);
		CmdBufLength = CmdBufLength - len;
	} else {
		// Check if we have too much garbage in the buffer (no EOL and buffer is full)
		if (CmdBufLength > sizeof(CmdBuf))
			CmdBufLength = 0;
	}
} // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

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
	Tchip *= (int32_t)(110000 - 30000);
	Tchip = Tchip / (int32_t)( TS_CAL2 - TS_CAL1);
	Tchip += 30000;
#ifdef SERIAL_DEBUG
	SerialPrint("A0: %d, A1: %d, A2: %d, A3: %d, A4: %d, A5: %d, A6: %d\n",
				AdcData[0], AdcData[1], AdcData[2], AdcData[3], AdcData[4], AdcData[5], AdcData[6]);
	SerialPrint("Cnt: %u, Vdd: %u, Tchip: %u, Vin: %u\n", AdcCount, Vdd, Tchip, Vin);
#endif
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
	SerialPrint("Serial debug!\n");
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
	HAL_TIM_PWM_ConfigChannel(phtim, &sConfigOC, channel);
	HAL_TIM_PWM_Start(phtim, channel);
} // PWM_Timer_Set

// Start timers
void StartTimers() {
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim14);
} // void StartTimers()

// Setup things before run
void Setup() {
#ifdef SERIAL_DEBUG
	DBGMCU->CR = 0x7;
#endif
	StartSerial();
	StartTimers();
	StartAdc();
} // void Setup()

// Process command
// Format: <Nchannel>:<Value>\n
void ProcessCmd(char *Cmd) {
	char *pSplit = memchr(Cmd, ':', MAX_COMMAND_LENGTH);
	int Chn, Val;

	// Wrong format, doing nothing
	if (!pSplit)
		return;
#ifdef SERIAL_DEBUG
	SerialPrint("Cmd: %s\n", Cmd);
#endif
	Chn = atoi(CmdNext);
	Val = atoi(pSplit + 1);

	// Check channel number and value
	if ((Chn < 0) || (Chn > MY_PWM_CHANNELS))
		return;
	if (Val < 0)
		Val = 0;
	if (Val > PWMRANGE)
		Val = PWMRANGE;
	Pwm[Chn] = Val;
	switch (Chn) {
	case 0:
		PWM_Timer_Set(&htim3, TIM_CHANNEL_1, Val, 0);
		break;
	case 1:
		PWM_Timer_Set(&htim14, TIM_CHANNEL_1, Val, 0);
		break;
	case 2:
		// Inverted PWM output (to spread switching current)
		PWM_Timer_Set(&htim3, TIM_CHANNEL_4, Val, 1);
		break;
	case 3:
		// Inverted PWM output (to spread switching current)
		PWM_Timer_Set(&htim3, TIM_CHANNEL_2, Val, 1);
		break;
	default:
		break;
	}
} // void ProcessCmd(char *Cmd)

int64_t LastUpdate = -9000;

// Main program loop
void Loop() {
	if ((HAL_GetTick() - LastUpdate) > REPORT_PERIOD) {
		UpdateThings();
		LastUpdate = HAL_GetTick();
		SerialPrint("0:%d\n1:%d\n2:%d\n3:%d\n4:%d\n5:%d\n6:%d\n",
				Vin, Vphr1, Vphr2, Vadc3, Vadc5, Vdd, Tchip);
	}
	if (FlagProcessCmd) {
#ifdef SERIAL_DEBUG
		SerialPrint("Processing command: %s", CmdNext);
#endif
		ProcessCmd(CmdNext);
		FlagProcessCmd = 0;
	}
} // void loop()
