#ifndef _BSP_NUCLEO_
#define _BSP_NUCLEO_

#include "stm32f767xx.h"
#include "stm32f7xx.h"

// LED1: green
#define LED1_PORT GPIOB
#define LED1_PIN  0

// LED2: blue
#define LED2_PORT GPIOB
#define LED2_PIN  7

// LED3: ref
#define LED3_PORT GPIOB
#define LED3_PIN  14

// User button
#define USER_BUTTON_PORT GPIOC
#define USER_BUTTON_PIN  13
#define USER_BUTTON_EXTI_LINE 13
#define USER_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define USER_BUTTON_IRQHANDLER EXTI15_10_IRQHandler

// ST-Link UART connection
#define STLINK_UART_TX_PORT GPIOD
#define STLINK_UART_TX_PIN 8
#define STLINK_UART_RX_PORT GPIOD
#define STLINK_UART_RX_PIN 9

#endif
