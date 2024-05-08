#pragma once

#include "stm32l432xx.h"

#define CLOCK_NONE    0b0000 // MCO output disabled, no clock on MCO
#define CLOCK_SYSCLK  0b0001 // SYSCLK system clock selected
#define CLOCK_MSI     0b0010 // MSI clock selected
#define CLOCK_HSI16   0b0011 // HSI16 clock selected
#define CLOCK_HSE     0b0100 // HSE clock selected
#define CLOCK_PLL     0b0101 // Main PLL clock selected
#define CLOCK_LSI     0b0110 // LSI clock selected
#define CLOCK_LSE     0b0111 // LSE clock selected
#define CLOCK_HSI48   0b1000 // Internal HSI48 clock selected

#define MCO_DIV_1   0b000 // MCO is divided by 1
#define MCO_DIV_2   0b001 // MCO is divided by 2
#define MCO_DIV_4   0b010 // MCO is divided by 4
#define MCO_DIV_8   0b011 // MCO is divided by 8
#define MCO_DIV_16  0b100 // MCO is divided by 16

void clock_HSEOn(void);
void clock_HSEOff(void);

void clock_PLLOn(void);
void clock_PLLOff(void);

void clock_MCO(uint8_t clk, uint8_t div);
