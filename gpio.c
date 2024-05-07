#include "gpio.h"

GPIO_TypeDef* getGPIO(uint8_t port)
{
  GPIO_TypeDef* gpio;
  switch (port)
  {
    case Port_A:
      gpio = GPIOA;
      break;

    case Port_B:
      gpio = GPIOB;
      break;

    #ifdef GPIOC
    case Port_C:
      gpio = GPIOC;
      break;
    #endif

    #ifdef GPIOD
    case Port_D:
      gpio = GPIOD;
      break;
    #endif

    #ifdef GPIOE
    case Port_E:
      gpio = GPIOE;
      break;
    #endif

    #ifdef GPIOF
    case Port_F:
      gpio = GPIOF;
      break;
    #endif

    #ifdef GPIOG
    case Port_G:
      gpio = GPIOG;
      break;
    #endif

    #ifdef GPIOH
    case Port_H:
      gpio = GPIOH;
      break;
    #endif


    #ifdef GPIOI
    case Port_I:
      gpio = GPIOI;
      break;
    #endif

    #ifdef GPIOJ
    case Port_J:
      gpio = GPIOJ;
      break;
    #endif

    #ifdef GPIOK
    case Port_K:
      gpio = GPIOK;
      break;
    #endif
  }

  return gpio;
}

void initGPIOPort(uint8_t gpio_port)
{
  RCC->AHB2ENR |= (0b1 << gpio_port);
}

/******************************* Output Pins *******************************/
void setOutputPin(uint8_t port_pin)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  initGPIOPort(port);

  GPIO->MODER &= ~(0b11 << (pin * 2)); // Reset pin mode
  GPIO->MODER |= (OUT << (pin * 2)); // Set pin mode to output

  GPIO->OTYPER &= ~(0b1 << pin); // Set pin as push-pull output

  GPIO->OSPEEDR &= ~(0b11 << (pin * 2)); // Reset pin output speed
  GPIO->OSPEEDR |= (OSPEED_HS << (pin * 2)); // Set pin output speed to very high speed (HS)
}

void setOutputPin_ospeed(uint8_t port_pin, uint8_t ospeed)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  initGPIOPort(port);

  GPIO->MODER &= ~(0b11 << (pin * 2)); // Reset pin mode
  GPIO->MODER |= (OUT << (pin * 2)); // Set pin mode to output

  GPIO->OTYPER &= ~(0b1 << pin); // Set pin as push-pull output

  GPIO->OSPEEDR &= ~(0b11 << (pin * 2)); // Reset pin output speed
  GPIO->OSPEEDR |= (ospeed << (pin * 2)); // Set pin output speed
}

void pinWrite(uint8_t port_pin, uint8_t state)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  if (state)
    GPIO->ODR |= (0b1 << pin); // Set state to ON
  else
    GPIO->ODR &= ~(0b1 << pin); // Set state to OFF
}

void pinToggle(uint8_t port_pin)
{
  pinRead(port_pin) ? pinWrite(port_pin, LOW) : pinWrite(port_pin, HIGH);
}

/******************************* Input Pins *******************************/
void setInputPin(uint8_t port_pin)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  initGPIOPort(port);

  GPIO->MODER &= ~(0b11 << (pin * 2)); // Reset pin mode
  GPIO->MODER |= (IN << (pin * 2)); // Set pin mode to input

  GPIO->PUPDR &= ~(0b11 << (pin * 2)); // Reset pin pull-up/pull-down mode
  GPIO->PUPDR |= (PUPD_PD << (pin * 2)); // Set pin to pull-down
}

void setInputPin_pupd(uint8_t port_pin, uint8_t pupd)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  initGPIOPort(port);

  GPIO->MODER &= ~(0b11 << (pin * 2)); // Reset pin mode
  GPIO->MODER |= (IN << (pin * 2)); // Set pin mode to input

  GPIO->PUPDR &= ~(0b11 << (pin * 2)); // Reset pin pull-up/pull-down mode
  GPIO->PUPDR |= (pupd << (pin * 2)); // Set pin to pull-down
}

uint8_t pinRead(uint8_t port_pin)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;
  GPIO_TypeDef* GPIO = getGPIO(port);

  if (GPIO->IDR & (0b1 << pin))
    return HIGH; // Pin is HIGH
  else
    return LOW; // Pin is LOW
}

/******************************* General I/O Pins *******************************/
void setPin(uint8_t port_pin, uint8_t state)
{
  if (state == IN)
    setInputPin(port_pin);
  else
    setInputPin(port_pin);
}

/******************************* Interrupt Pins *******************************/
void irqStartPin(uint8_t port_pin)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;

	RCC->APB2ENR |= 0b1; // Enable IRQ clock
  setInputPin(port_pin);
  
  uint8_t exti = pin / 4;
  uint8_t exti_pin = pin % 4;
  SYSCFG->EXTICR[exti] &= ~(0b111 << exti_pin * 4); // Reset EXTICR[X]
  SYSCFG->EXTICR[exti] |= (port << exti_pin * 4);

  // Enable appropriate NVIC
  __disable_irq();
  if (pin == 0)
    NVIC_EnableIRQ(EXTI0_IRQn);
  else if (pin == 1)
    NVIC_EnableIRQ(EXTI1_IRQn);
  else if (pin == 2)
    NVIC_EnableIRQ(EXTI2_IRQn);
  else if (pin == 3)
    NVIC_EnableIRQ(EXTI3_IRQn);
  else if (pin == 4)
    NVIC_EnableIRQ(EXTI4_IRQn);
  else if (pin < 10)
    NVIC_EnableIRQ(EXTI9_5_IRQn);
  else
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  __enable_irq();

  EXTI->IMR1 |= (0b1 << pin); // Enable the Interrupt Mask register
}

void irqEnablePinEdge(uint8_t port_pin, uint8_t edge)
{
  uint8_t pin = port_pin % 16;

  irqStartPin(port_pin);

  if (edge)
    EXTI->RTSR1 |= (1 << pin); // Enable rising trigger
  else
    EXTI->FTSR1 |= (1 << pin); // Enable falling trigger
}

void irqEnablePinRFE(uint8_t port_pin)
{
  uint8_t pin = port_pin % 16;

  irqStartPin(port_pin);
  
  EXTI->RTSR1 |= (1 << pin); // Enable rising trigger
  EXTI->FTSR1 |= (1 << pin); // Enable falling trigger
}

void irqDisablePinEdge(uint8_t port_pin, uint8_t edge)
{
  uint8_t pin = port_pin % 16;

  if (edge)
    EXTI->RTSR1 |= ~(1 << pin); // Disable rising trigger
  else
    EXTI->FTSR1 |= ~(1 << pin); // Disable falling trigger
}

void irqDisablePinRFE(uint8_t port_pin)
{
  uint8_t pin = port_pin % 16;

  EXTI->RTSR1 |= ~(1 << pin); // Disable rising trigger
  EXTI->FTSR1 |= ~(1 << pin); // Disable falling trigger
}

void irqStopPin(uint8_t port_pin)
{
  uint8_t port = port_pin / 16;
  uint8_t pin = port_pin % 16;

  uint8_t exti = pin / 4;
  uint8_t exti_pin = pin % 4;

  SYSCFG->EXTICR[exti] &= ~(0b111 << exti_pin * 4); // Reset EXTICR[X]

  EXTI->IMR1 |= (0b1 << pin); // Disable the Interrupt Mask register

  irqDisablePinRFE(port_pin);

  // Disable appropriate NVIC
  __disable_irq();
  if (pin == 0)
    NVIC_DisableIRQ(EXTI0_IRQn);
  else if (pin == 1)
    NVIC_DisableIRQ(EXTI1_IRQn);
  else if (pin == 2)
    NVIC_DisableIRQ(EXTI2_IRQn);
  else if (pin == 3)
    NVIC_DisableIRQ(EXTI3_IRQn);
  else if (pin == 4)
    NVIC_DisableIRQ(EXTI4_IRQn);
  else if (pin < 10)
    NVIC_DisableIRQ(EXTI9_5_IRQn);
  else
    NVIC_DisableIRQ(EXTI15_10_IRQn);
  __enable_irq();
}