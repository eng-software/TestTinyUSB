/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32h747xx.h"
#include "stm32h7xx_hal.h"

#include "board_api.h"
#include "tusb.h"
#include "board.h"

#define UART_DEV              USART1
#define UART_CLK_EN           __HAL_RCC_USART1_CLK_ENABLE


#ifdef UART_DEV
UART_HandleTypeDef UartHandle = {
  .Instance = UART_DEV,
  .Init = {
    .BaudRate = CFG_BOARD_UART_BAUDRATE,
    .WordLength = UART_WORDLENGTH_8B,
    .StopBits = UART_STOPBITS_1,
    .Parity = UART_PARITY_NONE,
    .HwFlowCtl = UART_HWCONTROL_NONE,
    .Mode = UART_MODE_TX_RX,
    .OverSampling = UART_OVERSAMPLING_16,
  }
};
#endif


//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+
void MX_USB_OTG_HS_HCD_Init(void);

void Error_Handler()
{
	while(1)
	{

	}
}

void hid_app_task(void);
void cdc_app_task(void);

/*------------- MAIN -------------*/
int main(void)
{
	HAL_Init();

	// Configurar PLL1 para 400 MHz e PLL3 para 60 MHz (USB)
	RCC_OscInitTypeDef RCC_OscInit = {0};
	RCC_ClkInitTypeDef RCC_ClkInit = {0};

	// Ativar HSE (exemplo: 25 MHz)
	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInit.HSEState = RCC_HSE_ON;
	HAL_RCC_OscConfig(&RCC_OscInit);

	// PLL1: 25 MHz HSE -> 400 MHz SYSCLK
	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLM = 5;       // 25 MHz /5 = 5 MHz
	RCC_OscInit.PLL.PLLN = 160;      // 5 MHz *160 = 800 MHz
	RCC_OscInit.PLL.PLLP = 2;        // 800 MHz /2 = 400 MHz (SYSCLK)
	HAL_RCC_OscConfig(&RCC_OscInit);

	// PLL3: 25 MHz HSE -> 60 MHz USB
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLM = 5;        // 25 MHz /5 = 5 MHz
	RCC_OscInit.PLL.PLLN = 60;       // 5 MHz *60 = 300 MHz
	RCC_OscInit.PLL.PLLP = 5;        // 300 MHz /5 = 60 MHz (USB)
	HAL_RCC_OscConfig(&RCC_OscInit);

	// Configurar divisores de clock
	RCC_ClkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 400 MHz
	RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;        // 400 MHz
	RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;         // 200 MHz
	RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV2;         // 200 MHz
	HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_4);

	// Selecionar PLL3 como clock do USB
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	SystemCoreClockUpdate();
	//--------------------------------------------------------------------------

	// Exemplo para ULPI no STM32H747 (verifique o seu board!)
	// ULPI_CLK: PH4 (AF10)
	// ULPI_D0-D7: PH0-PH7 (AF10)
	// ULPI_DIR: PI11 (AF10)
	// ULPI_NXT: PI10 (AF10)
	// ULPI_STP: PC0 (AF10)

	GPIO_InitTypeDef GPIO_Init = {0};

	// Ativar clocks dos GPIOs
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Configurar ULPI_CLK (PH4)
	GPIO_Init.Pin = GPIO_PIN_4;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF10_OTG1_HS;
	HAL_GPIO_Init(GPIOH, &GPIO_Init);

	// Configurar ULPI_D0-D7 (PH0-PH7)
	GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
	                GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOH, &GPIO_Init);

	// Configurar ULPI_DIR (PI11), ULPI_NXT (PI10)
	GPIO_Init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	HAL_GPIO_Init(GPIOI, &GPIO_Init);

	// Configurar ULPI_STP (PC0)
	GPIO_Init.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOC, &GPIO_Init);
	//---------------------------------------------------------------------

	MX_USB_OTG_HS_HCD_Init();

	SysTick_Config(SystemCoreClock / 1000);

#ifdef UART_DEV
	UART_CLK_EN();
	HAL_UART_Init(&UartHandle);
#endif

	tuh_init(BOARD_TUH_RHPORT); // Inicializa TinyUSB

	// Configura a IRQ do USB OTG HS
	HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);  // Prioridade média
	HAL_NVIC_EnableIRQ(OTG_HS_IRQn);          // Habilita a interrupção

	printf("System start success\n");

	while (1)
	{
		tuh_task();               // Processa eventos USB
		hid_app_task();
		cdc_app_task();

	}
}

void OTG_HS_IRQHandler(void)
{
	tuh_int_handler(BOARD_TUH_RHPORT);  // BOARD_TUH_RHPORT = 0
}

HCD_HandleTypeDef hhcd;

void MX_USB_OTG_HS_HCD_Init(void)
{
  // Configurar o Host Controller
  hhcd.Instance = USB_OTG_HS;
  hhcd.Init.Host_channels = 16;
  hhcd.Init.speed = HCD_SPEED_HIGH;
  hhcd.Init.phy_itface = USB_OTG_ULPI_PHY; // Modo ULPI
  hhcd.Init.Sof_enable = DISABLE;

  // Inicializar o Host Controller
  if (HAL_HCD_Init(&hhcd) != HAL_OK) {
    Error_Handler();
  }

  // Verificar se o PHY está pronto (USB3320)
  if (HAL_HCD_Start(&hhcd) != HAL_OK)
  {
    Error_Handler();
  }

  // O Vendor ID do USB3320 será detectado automaticamente pelo driver.
  // Para confirmar, verifique o sinal no barramento ULPI com um analisador lógico.
}

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void)
{
	HAL_IncTick();
	system_ticks++;
}

uint32_t board_millis(void)
{
	return system_ticks;
}

void HardFault_Handler(void)
{
	__asm("BKPT #0\n");
}

int board_uart_read(uint8_t *buf, int len)
{
	(void) buf;
	(void) len;
	return 0;
}

int board_uart_write(void const *buf, int len)
{
#ifdef UART_DEV
	HAL_UART_Transmit(&UartHandle, (uint8_t * )(uintptr_t)buf, len, 0xffff);
	return len;
#else
	(void) buf; (void) len;
	return -1;
#endif
}
