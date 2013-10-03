/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright � 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART3 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART3_TX_PIN        GPIO_Pin_8
#define UART3_RX_PIN        GPIO_Pin_9
#define UART3_GPIO          GPIOD
#define UART3_TX_PINSOURCE  GPIO_PinSource8
#define UART3_RX_PINSOURCE  GPIO_PinSource9

#define UART3_BUFFER_SIZE   2048

// Receive buffer, circular DMA
volatile uint8_t rx3Buffer[UART3_BUFFER_SIZE];
uint32_t rx3DMAPos = 0;

volatile uint8_t  tx3Buffer[UART3_BUFFER_SIZE];
volatile uint16_t tx3BufferTail = 0;
volatile uint16_t tx3BufferHead = 0;

volatile uint8_t  tx3DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART3 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart3TxDMA(void)
{
	if ((tx3DmaEnabled == true) || (tx3BufferHead == tx3BufferTail))  // Ignore call if already active or no new data in buffer
        return;

    DMA1_Stream3->M0AR = (uint32_t)&tx3Buffer[tx3BufferTail];

    if (tx3BufferHead > tx3BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA1_Stream3, tx3BufferHead - tx3BufferTail);
	    tx3BufferTail = tx3BufferHead;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA1_Stream3, UART3_BUFFER_SIZE - tx3BufferTail);
	    tx3BufferTail = 0;
    }

    tx3DmaEnabled = true;

    DMA_Cmd(DMA1_Stream3, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART3 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA1_Stream3_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

    tx3DmaEnabled = false;

    uart3TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Initialization
///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 0 };

void openLogListenerCB(evr_t e)
{
    if (expandEvr)
        openLogPrintF("EVR-%s %8.3fs %s (%04x)\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        openLogPrintF("EVR:%08x %04x %04x\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////

void openLogInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_PinAFConfig(UART3_GPIO, UART3_TX_PINSOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_GPIO, UART3_RX_PINSOURCE, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin   = UART3_TX_PIN | UART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART3, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA1_Stream1);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx3Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = UART3_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Stream1, ENABLE);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    rx3DMAPos = DMA_GetCurrDataCounter(DMA1_Stream1);

    // Transmit DMA
    DMA_DeInit(DMA1_Stream3);

  //DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx3Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  //DMA_InitStructure.DMA_BufferSize         = UART3_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  //DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA1_Stream3, 0);

    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART3, ENABLE);

    evrRegisterListener(openLogListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Available
///////////////////////////////////////////////////////////////////////////////

uint16_t openLogAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Stream1) != rx3DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Read
///////////////////////////////////////////////////////////////////////////////

uint8_t openLogRead(void)
{
    uint8_t ch;

    ch = rx3Buffer[UART3_BUFFER_SIZE - rx3DMAPos];
    // go back around the buffer
    if (--rx3DMAPos == 0)
	    rx3DMAPos = UART3_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t openLogReadPoll(void)
{
    while (!openLogAvailable()); // wait for some bytes
    return openLogRead();
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Write
///////////////////////////////////////////////////////////////////////////////

void openLogWrite(uint8_t ch)
{
    tx3Buffer[tx3BufferHead] = ch;
    tx3BufferHead = (tx3BufferHead + 1) % UART3_BUFFER_SIZE;

    uart3TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Open Log Print
///////////////////////////////////////////////////////////////////////////////

void openLogPrint(char *str)
{
    while (*str)
    {
    	tx3Buffer[tx3BufferHead] = *str++;
    	tx3BufferHead = (tx3BufferHead + 1) % UART3_BUFFER_SIZE;
    }

	uart3TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// OpenLog Print Formatted - Print formatted string to Telemetry Port
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void openLogPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	openLogPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
