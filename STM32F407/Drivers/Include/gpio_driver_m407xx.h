/**
  ************************************************************************
  * @file       gpio_driver_m407xx.h
  * @author     Aditya Mall
  * @date       2018-06-25
  * @copyright  (C) 2018 Aditya Mall.
  * @brief      STM32F407xx GPIO Driver Header File
  *
  *             This file contains:
  *              - Macros for GPIO pin Initialization
  *              - Macros for enabling & disabling Clock at GPIO Ports
  *              - Data Structures for GPIO Pin Initialization
  *              - Driver exposed API Function Prototypes
  *              - Helper function prototypes fop APIs
  *              - Driver exposed API Interupts Prototypes
  *
  ************************************************************************
  * @attention
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  ************************************************************************
  */

/**
  *
  * @version 1.0 - First release
  *	@brief   Compact GPIO driver for STM32F407VGT ARM Cortex M(TM) based Microcontroler,
  *	         by ST Microelectronics.
  *
  * @dependencies stm32f407xx.h, system_stm32f4xx.c, startup_stm32f407xx.s
  *
  */

#ifndef __GPIO_DRIVER_M407_H
#define __GPIO_DRIVER_M407_H


	/* MCU specific header files */
#include "stm32f407xx.h"


/******************************************************************************/
/*                                                                            */
/*               1. Macros for GPIO Pin Initialization                        */
/*                                                                            */
/******************************************************************************/

/**
  * @brief GPIO PORT Address values as per MCU header file.
  *        All Values are defined in MCU header file.
  */
  /* All GPIO Port address defines                         */
#define PORT_A                            GPIOA
#define PORT_B                            GPIOB
#define PORT_C                            GPIOC
#define PORT_D                            GPIOD
#define PORT_E                            GPIOE
#define PORT_F                            GPIOF
#define PORT_G                            GPIOG
#define PORT_H                            GPIOH
#define PORT_I                            GPIOI
#define PORT_J                            GPIOJ
#define PORT_K                            GPIOK

/* Optional defines that can be used                      */
#define A_ADDR                            PORT_A
#define B_ADDR                            PORT_B
#define C_ADDR                            PORT_C
#define D_ADDR                            PORT_D
#define E_ADDR                            PORT_E
#define F_ADDR                            PORT_F
#define G_ADDR                            PORT_G
#define H_ADDR                            PORT_H
#define I_ADDR                            PORT_I
#define J_ADDR                            PORT_J
#define K_ADDR                            PORT_K

/* GPIO MODER register defines for mode selection          */
#define MODER_INPUT                       ((uint32_t)0x00U)
#define MODER_OUTPUT                      ((uint32_t)0x01U)                 
#define MODER_ALT                         ((uint32_t)0x02U)
#define MODER_ANALOG                      ((uint32_t)0x03U)

/* GPIO OTYPER register defines for output type selection  */
#define OTYPER_PUSHPULL                   ((uint16_t)0x00U)
#define OTYPER_OPENDRAIN                  ((uint16_t)0x01U)

/* GPIO OSPEEDR register for speed selection               */
#define OSPEEDR_LOW                       ((uint32_t)0x00U)
#define OSPEEDR_MEDIUM                    ((uint32_t)0x01U)
#define OSPEEDR_HIGH                      ((uint32_t)0x02U)
#define OSPEEDR_VERY_HIGH                 ((uint32_t)0x03U)

/* GPIO PUPDR register for pull-up/pull-down configuration */
#define PUPDR_NO_UP_DOWN                  ((uint32_t)0x00U)
#define PUPDR_PULL_UP                     ((uint32_t)0x01U)
#define PUPDR_PULL_DOWN                   ((uint32_t)0x02U)
#define PUPDR_RESERVED                    ((uint32_t)0x03U)

/**
  * @brief GPIO AFRL and AFRH register for setting alternate,
  *        low and high registers.
  */
#define ALT_AF0                           ((uint32_t)0x00U)
#define ALT_AF1                           ((uint32_t)0x01U)
#define ALT_AF2                           ((uint32_t)0x02U)
#define ALT_AF3                           ((uint32_t)0x03U)
#define ALT_AF4                           ((uint32_t)0x04U)
#define ALT_AF5                           ((uint32_t)0x05U)
#define ALT_AF6                           ((uint32_t)0x06U)
#define ALT_AF7                           ((uint32_t)0x07U)
#define ALT_AF8                           ((uint32_t)0x08U)
#define ALT_AF9                           ((uint32_t)0x09U)
#define ALT_AF10                          ((uint32_t)0x0AU)
#define ALT_AF11                          ((uint32_t)0x0BU)
#define ALT_AF12                          ((uint32_t)0x0CU)
#define ALT_AF13                          ((uint32_t)0x0DU)
#define ALT_AF14                          ((uint32_t)0x0EU)
#define ALT_AF15                          ((uint32_t)0x0FU)

  /* System default alternate function register define       */
#define ALT_SYSTEM                        ALT_AF0

/* TIMER alternate function register defines               */
#define ALT_TIM1_TIM2                     ALT_AF1
#define ALT_TIM3_TO_TIM5                  ALT_AF2
#define ALT_TIM8_TO_TIM11                 ALT_AF3
#define ALT_TIM12_TO_TIM14                ALT_AF9

/* I2C and SPI alternate function register defines         */
#define ALT_I2C1                          ALT_AF4
#define ALT_I2C2                          ALT_AF4
#define ALT_I2C3                          ALT_AF4
#define ALT_SPI1                          ALT_AF5
#define ALT_SPI2                          ALT_AF5
#define ALT_SPI3                          ALT_AF6

/* USART alternate function register defines               */
#define ALT_USART1_TO_USART3              ALT_AF7
#define ALT_USART4_TO_USART6              ALT_AF8

/* CAN bus alternate function register defines             */
#define ALT_CAN1_CAN2                     ALT_AF9

/* USB OTG alternate function register defines             */
#define ALT_OTG_FS                        ALT_AF10
#define ALT_OTG_HS                        ALT_AF10
#define ALT_OTG_HS_1                      ALT_AF12


//            !! #Documentation required !!
#define ALT_ETH                           ALT_AF11
#define ALT_FSMC                          ALT_AF12
#define ALT_SDIO                          ALT_AF12
#define ALT_DCMI                          ALT_AF13
#define ALT_RESERVED                      ALT_AF14
#define EVENTOUT                          ALT_AF15  

/**
  * @brief SYSCFG EXTICR register defines
  */
#define PA                                ((uint32_t)0x00U)
#define PB                                ((uint32_t)0x01U)
#define PC                                ((uint32_t)0x02U)
#define PD                                ((uint32_t)0x03U)
#define PE                                ((uint32_t)0x04U)
#define PF                                ((uint32_t)0x05U)
#define PG                                ((uint32_t)0x06U)
#define PH                                ((uint32_t)0x07U)
#define PI                                ((uint32_t)0x08U)



  /******************************************************************************/
  /*                                                                            */
  /*         2. Macros for enabling & disabling Clock at GPIO Ports             */
  /*                                                                            */
  /******************************************************************************/

  /**
	* @brief   Port Base Address intialization
	*          struct RCC defined in MCU specific header file.
	*/
	/* Enable RCC clock for GPIO PORTS  */
#define _HAL_RCC_GPIOA_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 0))
#define _HAL_RCC_GPIOB_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 1))
#define _HAL_RCC_GPIOC_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 2))
#define _HAL_RCC_GPIOD_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 3))
#define _HAL_RCC_GPIOE_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 4))
#define _HAL_RCC_GPIOF_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 5))
#define _HAL_RCC_GPIOG_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 6))
#define _HAL_RCC_GPIOH_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 7))
#define _HAL_RCC_GPIOI_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 8))
#define _HAL_RCC_GPIOJ_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 9))
#define _HAL_RCC_GPIOK_CLK_ENABLE()                 (RCC->AHB1ENR |= (1 << 10))

/* Disable RCC clock for GPIO PORTS */
#define _HAL_RCC_GPIOA_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 0))
#define _HAL_RCC_GPIOB_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 1))
#define _HAL_RCC_GPIOC_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 2))
#define _HAL_RCC_GPIOD_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 3))
#define _HAL_RCC_GPIOE_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 4))
#define _HAL_RCC_GPIOF_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 5))
#define _HAL_RCC_GPIOG_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 6))
#define _HAL_RCC_GPIOH_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 7))
#define _HAL_RCC_GPIOI_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 8))
#define _HAL_RCC_GPIOJ_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 9))
#define _HAL_RCC_GPIOK_CLK_DISABLE()                (RCC->AHB1ENR &= ~(1 << 10))



/******************************************************************************/
/*                                                                            */
/*               3. Data Structures for GPIO Pin Initialization               */
/*                                                                            */
/******************************************************************************/

/**
  * @brief GPIO user defined pin configuration data structure
  */
typedef struct
{
	uint32_t PIN;             /* Specifies the GPIO pin to be configured      */
	uint32_t MODE;            /* Specifies the Operating mode of the GPIO pin */
	uint32_t OPTYPE;          /* Specifies the output type  of the GPIO pin   */
	uint32_t SPEED;           /* Specifies the Operating speed of the GPIO    */
	uint32_t PUPD;            /* Specifies push pull, pull uo and open drain  */
	uint32_t ALT;             /* Specifies the alternate functionalties       */
}pin_config_t;

/**
  * @brief Interrupt Edge selection enum
  */
typedef enum
{
	INT_RISING_EDGE,          /* Rising Edge, low to high of GPIO pin          */
	INT_FALLING_EDGE,         /* Falling Edge, high to low of GPIO pin         */
	INT_RISING_FALLING_EDGE   /* Both Rising and Falling edges                 */
}edge_select_t;


/******************************************************************************/
/*                                                                            */
/*               4. Driver exposed API Function Prototypes                    */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Intializes GPIO pin
  * @param  *GPIOx         : GPIO port base address
  * @param  *gpio_pin_conf : pointer to the gpio pin config structure.
  * @retval None.
  */
void _hal_gpio_init(GPIO_TypeDef *GPIOx, pin_config_t *gpio_pin_conf);


/**
  * @brief  Reads value from the GPIO pin
  * @param  *GPIOx  : GPIO port base address
  * @param  pin_no  : GPIO pin number
  * @retval uint8_t : Read Value
  */
uint8_t _hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no);


/**
  * @brief  Writes value to the GPIO pin
  * @param  *GPIOx : GPIO port base address
  * @param  pin_no : GPIO pin number
  * @param  val    : value to be written
  * @retval None
  */
void _hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val);


/**
  * @brief  Set the alternate functinonality to the given pin.
  * @param  *GPIOx         : GPIO port base address
  * @param  pin_no         : GPIO pin number
  * @param  alt_func_value : value to be written
  * @retval None
  */
void _hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t alt_func_value);

/**
  * @brief  Atomi bit set reset for GPIO pin
  * @param  *GPIOx  : GPIO port base address
  * @param  pin_no  : GPIO pin number
	* @param  val     : if 1 then the ODR bit is set.
	* @retval None
  */
void _hal_gpio_bit_set_reset_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val);




/******************************************************************************/
/*                                                                            */
/*               5. Helper function prototypes for APIs                       */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Set the mode register for a specific pin.
  * @param  *GPIOx   : GPIO port base address
  * @param  pin_no   : GPIO pin number
  * @param  pin_mode : Mode to be configured
  * @retval none
  */
static void _hf_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_mode);


/**
   * @brief  Configures the output type of a pin
   * @param  *GPIOx     : GPIO Port Base address
   * @param  pin_no     : GPIO pin number
   * @param  pin_optype : output type to be configured with
   * @retval None
   */
static void _hf_gpio_configure_pin_optype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t pin_optype);

/**
  * @brief  Configures the speed of a pin
  * @param  *GPIOx    : GPIO Port Base address
  * @param  pin_no    : GPIO pin number
  * @param  pin_speed : value of the speed
  * @retval None
  */
static void _hf_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_speed);

/**
  * @brief  Activates the internall pull up or pull down resistors
  * @param  *GPIOx   : GPIO Port Base address
  * @param  pin_no   : GPIO pin number
  * @param  pin_pupd : specifies which resistor to activate
  * @retval None
  */
static void _hf_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_pupd);



/******************************************************************************/
/*                                                                            */
/*               6. Driver exposed API Interupts Prototypes                   */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Configure the interrupt for a given pin number
  * @param  pin_no   : GPIO pin number
  * @param  pin_port : Port of the register
  * @param  edge_sel : Triggering edge slection value of type "int_edge_sel_t"
  * @retval None
  */
void _hal_gpio_configure_interrupt(uint16_t pin_no, uint8_t pin_port, edge_select_t edge_sel);


/**
  * @brief  Enable the interrupt for a give pin number and irq number
  * @param  pin_no : GPIO pin number
  * @param  irq_no : irq_number to be enabled in NVIC
  * @retval None
  */
void _hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);


/**
  * @brief  Clear the interrupt pending bit if set
  * @param  pin_no : GPIO pin number
  * @retval None
  */
void _hal_gpio_clear_interrupt(uint16_t pin_no);

#endif

/**************************************************** END OF FILE *************************************************/


















