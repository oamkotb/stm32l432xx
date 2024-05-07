#pragma once

#include "stm32l432xx.h"

#define IN  0b00
#define OUT 0b01

#define LOW  0x00
#define HIGH 0xFF

#define OSPEED_LS 0b00 // Low speed
#define OSPEED_MS 0b01 // Medium speed
#define OSPEED_FS 0b10 // High speed
#define OSPEED_HS 0b11 // Very high speed

#define PUPD_NP 0b00   // No pull-up/pull-down
#define PUPD_PU 0b01   // Pull-up
#define PUPD_PD 0b10   // Pull-down

#define FALLING_EDGE 0
#define RISING_EDGE  1

#define Port_A 0
enum port_A { PA0 = 0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15 };

#define Port_B 1
enum port_B { PB0 = 16, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15 };

#ifdef GPIOC
#define Port_C 2
enum port_C { PC0 = 32, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15 };
#endif

#ifdef GPIOD
#define Port_D 3
enum port_D { PD0 = 48, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15 };
#endif

#ifdef GPIOE
#define Port_E 4
enum port_E { PE0 = 64, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15 };
#endif

#ifdef GPIOF
#define Port_F 5
enum port_F { PF0 = 80, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15 };
#endif

#ifdef GPIOG
#define Port_G 6
enum port_G { PG0 = 96, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15 };
#endif

#ifdef GPIOH
#define Port_H 7
enum port_H { PH0 = 112, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15 };
#endif

#ifdef GPIOI
#define Port_I 8
enum port_I { PI0 = 128, PI1, PI2, PI3, PI4, PI5, PI6, PI7, PI8, PI9, PI10, PI11, PI12, PI13, PI14, PI15 };
#endif

#ifdef GPIOJ
#define Port_J 9
enum port_J { PJ0 = 144, PJ1, PJ2, PJ3, PJ4, PJ5, PJ6, PJ7, PJ8, PJ9, PJ10, PJ11, PJ12, PJ13, PJ14, PJ15 };
#endif

#ifdef GPIOK
#define Port_K 10
enum port_K { PK0 = 160, PK1, PK2, PK3, PK4, PK5, PK6, PK7, PK8, PK9, PK10, PK11, PK12, PK13, PK14, PK15 };
#endif

void setOutputPin(uint8_t port_pin);
void setOutputPin_ospeed(uint8_t port_pin, uint8_t ospeed);

void pinWrite(uint8_t port_pin, uint8_t state);
void pinToggle(uint8_t port_pin);

void setInputPin(uint8_t port_pin);
void setInputPin_pupd(uint8_t port_pin, uint8_t pupd);

void setPin(uint8_t port_pin, uint8_t state);

uint8_t pinRead(uint8_t port_pin);

void irqStartPin(uint8_t port_pin);
void irqEnablePinEdge(uint8_t port_pin, uint8_t edge); // Enable either rising or falling trigger
void irqEnablePinRFE(uint8_t port_pin); // Enable both rising & falling trigger
void irqStopPin_edge(uint8_t port_pin); // Disable either rising or falling trigger
void irqStopPin_rfe(uint8_t port_pin); // Disable both rising & falling trigger
void irqStopPin(uint8_t port_pin);
