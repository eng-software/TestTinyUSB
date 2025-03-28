/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   name: STM32 H743 Nucleo
   url: https://www.st.com/en/evaluation-tools/nucleo-h743zi.html
*/

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define UART_DEV              USART1
#define UART_CLK_EN           __HAL_RCC_USART1_CLK_ENABLE

// VBUS Sense detection
#define OTG_HS_VBUS_SENSE     0


//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void SystemClock_Config(void) {
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  // Configure HSE (25 MHz)
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

	  // PLL1: System Clock (400 MHz)
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 5;          // 25 MHz /5 = 5 MHz
	  RCC_OscInitStruct.PLL.PLLN = 160;        // 5 MHz *160 = 800 MHz (VCO)
	  RCC_OscInitStruct.PLL.PLLP = 2;          // 800 MHz /2 = 400 MHz (SYSCLK)
	  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	  HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  // PLL3: USB Clock (48 MHz)
	  PeriphClkInitStruct.PLL3.PLL3M = 25;      // 25 MHz /25 = 1 MHz
	  PeriphClkInitStruct.PLL3.PLL3N = 96;      // 1 MHz *96 = 96 MHz (VCO)
	  PeriphClkInitStruct.PLL3.PLL3P = 2;       // 96 MHz /2 = 48 MHz (USB clock)
	  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
	  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;

	  // Assign clocks to peripherals
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3; // USB uses PLL3 (48 MHz)
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	  // Configure CPU, AHB, APB clocks
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
	                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 400 MHz
	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;           // HCLK = 200 MHz
	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;          // APB3 = 100 MHz
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;          // APB1 = 100 MHz
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;          // APB2 = 100 MHz
	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}


static inline void board_init2(void) {
  // For this board does nothing
}

void board_vbus_set(uint8_t rhport, bool state) {}

#ifdef __cplusplus
 }
#endif

#endif
