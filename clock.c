#include "clock.h"
#include "gpio.h"

void clock_HSEOn(void)
{
	RCC->CR |= (0b1 << 16); // Enable HSE clock
	while (!(RCC->CR & (0b1 << 17))); // Wait for HSE clock ready
}

void clock_HSEOff(void)
{
  RCC->CR &= ~(0b1 << 16); // Disable HSE clock
}

void clock_PLLOn(void)
{
	RCC->CR |= (0b1 << 24); // Enable PLL
	while (!(RCC->CR & (0b1 << 25))); // Wait for main PLL ready
}

void clock_PLLOff(void)
{
	RCC->CR &= ~(0b1 << 24); // Disable PLL
}

void clock_MCO(uint8_t clock, uint8_t div)
{
	setAltPin(PA8);

  RCC->CFGR &= ~(0b111 << 28);
  RCC->CFGR &= (0b111 << 28); // Select MCOSEL division

  RCC->CFGR &= ~(0b1111 << 24);
	RCC->CFGR |= (clock << 24); // Set MCOSEL to READ clock
}