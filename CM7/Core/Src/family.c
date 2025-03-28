/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019
 *    William D. Jones (thor0505@comcast.net),
 *    Ha Thach (tinyusb.org)
 *    Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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
   manufacturer: STMicroelectronics
*/

#include "stm32h7xx_hal.h"
#include "board_api.h"

void Error_Handler(void) { }

typedef struct {
  GPIO_TypeDef* port;
  GPIO_InitTypeDef pin_init;
  uint8_t active_state;
} board_pindef_t;

#include "board.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

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
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

// Despite being call USB2_OTG_FS on some MCUs
// OTG_FS is marked as RHPort0 by TinyUSB to be consistent across stm32 port
void OTG_FS_IRQHandler(void) {
  tusb_int_handler(0, true);
}

// Despite being call USB1_OTG_HS on some MCUs
// OTG_HS is marked as RHPort1 by TinyUSB to be consistent across stm32 port
void OTG_HS_IRQHandler(void) {
  tusb_int_handler(1, true);
}

#ifdef TRACE_ETM
void trace_etm_init(void) {
  // H7 trace pin is PE2 to PE6
  GPIO_InitTypeDef  gpio_init;
  gpio_init.Pin       = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  gpio_init.Mode      = GPIO_MODE_AF_PP;
  gpio_init.Pull      = GPIO_PULLUP;
  gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOE, &gpio_init);

  // Enable trace clk, also in D1 and D3 domain
  DBGMCU->CR |= DBGMCU_CR_DBG_TRACECKEN | DBGMCU_CR_DBG_CKD1EN | DBGMCU_CR_DBG_CKD3EN;
}
#else
  #define trace_etm_init()
#endif

void board_init(void) {
	HAL_Init();
  // Implemented in board.h
  SystemClock_Config();

  // Enable USB OTG HS peripheral clock
  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();


  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  // ULPI_CLK (PI5) - Alternate Function AF10 (USB_HS_ULPI)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  // ULPI_DIR (PI11)
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  // ULPI_NXT (PI10)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  // ULPI_STP (PC0)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // ULPI_DATA0 (PA3)
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ULPI_DATA1 (PB0)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA2 (PB1)
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA3 (PB10)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA4 (PB11)
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA5 (PB12)
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA6 (PB13)
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ULPI_DATA7 (PH4)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);


  // --- USART1 Pins ---
  // PA9: USART1_TX, PA10: USART1_RX
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;       // Adjust to GPIO_PULLUP if needed
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  // Alternate Function 7 for USART1
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  #ifdef USB_OTG_FS_PERIPH_BASE
  NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
  #endif

  NVIC_SetPriority(OTG_HS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif


#ifdef UART_DEV
  UART_CLK_EN();
  HAL_UART_Init(&UartHandle);
#endif
  HAL_PWREx_EnableUSBVoltageDetector();

  board_init2(); // optional init

#if CFG_TUH_ENABLED
  board_vbus_set(BOARD_TUH_RHPORT, 1);
#endif

}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
#ifdef PINID_LED
  board_pindef_t* pindef = &board_pindef[PINID_LED];
  GPIO_PinState pin_state = state == pindef->active_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(pindef->port, pindef->pin_init.Pin, pin_state);
#else
  (void) state;
#endif
}

uint32_t board_button_read(void) {
#ifdef PINID_BUTTON
  board_pindef_t* pindef = &board_pindef[PINID_BUTTON];
  return pindef->active_state == HAL_GPIO_ReadPin(pindef->port, pindef->pin_init.Pin);
#else
  return 0;
#endif
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
  uint32_t* id32 = (uint32_t*) (uintptr_t) id;
  uint8_t const len = 12;

  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];

  return len;
}

int board_uart_read(uint8_t *buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len) {
#ifdef UART_DEV
  HAL_UART_Transmit(&UartHandle, (uint8_t * )(uintptr_t)
  buf, len, 0xffff);
  return len;
#else
  (void) buf; (void) len;
  return -1;
#endif
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  HAL_IncTick();
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
//void _init(void) {
//}
