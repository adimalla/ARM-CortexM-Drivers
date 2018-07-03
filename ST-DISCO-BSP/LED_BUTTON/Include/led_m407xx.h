/**
  ***************************************************************************
  * @file       led_m407xx.h
  * @author     Aditya Mall
  * @date       2018-07-03
  * @copyright  (C) 2018 Aditya Mall.
  * @brief      LED and USER BUTTON, Board Support Package Header file, 
  *             for ST-Discovery M407G. 
  *
  *             This file contains:
  *              - Macros for LED pin defines and USER BUTTON
  *              - Function prototypes for LED and USER BUTTON API. 
  *
  ***************************************************************************
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
  * For any queries and comments mail me:-
  * @email <aditya.mall1990@gmail.com>
  *
  *****************************************************************************
  */

/**
  *****************************************************************************
  * @verbatim
  *              !! HOW TO USE THIS PACKAGE !!
  * 
  * This board support package contains total of 6 functions for initializing On 
  * Board LED and USER button on STM-Discovery-F407. The board contains 4 User LEDs
  * and 1 USER button. The Other additional LEDs and Button are Comms LED and Power 
  * LED and RESET Button.
  *
  * USER LED | PINS    |USER BUTTON | PIN
  *          |         |            | 
  * ORANGE   | PD13    |USER BUTTON | PA0
  * GREEN    | PD12
  * BLUE     | PD15
  * RED      | PD14
  * 
  * LED FUNCTION CALLS :-
  *   - void led_init(void);  
  *     Description: 
  *                  Initialize and configure led pin and registers and enable clock 
  *                  to corresponding led GPIO ports. 
  *     
  *   - void led_on(GPIO_TypeDef *GPIOx, uint16_t pin_no);
  *     Description: 
  *                  Set bit the for the correponding board LED
  *                  1) Arg: Base address of the GPIO PORT.
  *                  2) Arg: Pin Number of the LED
  *     Example:     
  *                  led_on(GPIOA, 13);
  *
  *   - void led_off(GPIO_TypeDef *GPIOx, uint16_t pin_no);
  *     Description: 
  *                  Reset bit the for the correponding board LED
  *                  1) Arg: Base address of the GPIO PORT.
  *                  2) Arg: Pin Number of the LED
  *     Example:     
  *                  led_off(GPIOD, 13);
  *
  * BUTTON FUNCTION CALLS :-
  *   - void button_init(GPIO_TypeDef *GPIOx, uint16_t pin_no);
  *     Description:
  *                  Initailze the user button and enable corresponding clock
  *                  1) Arg: Base address of the GPIO PORT.
  *                  2) Arg: Pin Number of the BUTTON
  *     Example:
  *                  button_init(GPIOA, 0);
  *
  *   - uint8_t button_read(GPIO_TypeDef *GPIOx, uint16_t pin_no);
  *     Description:
  *                  Read the input state of the button
  *                  1) Arg: Base address of the GPIO PORT.
  *                  2) Arg: Pin Number of the BUTTON
  *                  3) ret: Returns 1 for HIGH.
  *     Example:
  *                  int val = button_read(GPIOA, 0);
  *
  *   - void button_interrupt_init(GPIO_TypeDef *GPIOx, uint16_t pin_no);
  *     Description:
  *                  Configures and enables the button as interrupt
  *                  1) Arg: Base address of the GPIO PORT.
  *                  2) Arg: Pin Number of the BUTTON
  *     Note:       
  *                  Correspoding interrupt handler has to be implemented.
  *                  Default: void EXTI0_IRQHandler(void);
  *     Example:
  *                  button_interrupt_init(GPIOA, 0);
  *
  * @endverbatim
  *******************************************************************************
  */
    
/**
  *
  * @version 1.0 - First release
  * @brief   LED and Button Board support package interface for STM-Discovery-F407,
  *          by ST Microelectronics
  *
  * @dependencies gpio_driver_m407xx.h  (Aditya Mall)
  *               gpio_driver_m407xx.c  (Aditya Mall) 
  *               system_stm32f4xx.c    (Vendor Specific) 
  *               startup_stm32f407xx.s (Vendor Specific)
  *
  */


#ifndef _ONBOARD_LED_H
#define _ONBOARD_LED_H


/**
  * @brief  Header file for gpio driver
  */
#include "gpio_driver_m407xx.h"


/******************************************************************************/
/*                                                                            */
/*              1. Macros for LED & USER BUTTON pin defines                    */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  on board LED and USER BUTTON.
  */

/* GPIO PORT D for on board LEDs STM-Discovery 407      */
#define PD12           (12U)
#define PD13           (13U)
#define PD14           (14U)
#define PD15           (15U)

#define LED_GREEN      PD12
#define LED_ORANGE     PD13
#define LED_RED        PD14
#define LED_BLUE       PD15


/* GPIO PORT A for onboard USER BUTTON STM-Discovery 407 */
#define PA0             (0U)

#define USER_BUTTON     PA0



/******************************************************************************/
/*                                                                            */
/*                     1. Led API Function Prototypes                         */
/*                                                                            */
/******************************************************************************/


/**
  * @brief  Initialize the LEDs 
  * @param  None
  * @retval None
  */
void led_init(void);

/**
  * @brief  Turns ON the led which is connected on the given pin  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of the LED
  * @retval None
  */
void led_on(GPIO_TypeDef *GPIOx, uint16_t pin_no);


/**
  * @brief  Turns OFF the led which is connected on the given pin  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of the LED
  * @retval None
  */
void led_off(GPIO_TypeDef *GPIOx, uint16_t pin_no);


/**
  * @brief  Toggels the led which is connected on the given pin  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of the LED
  * @retval None
  */
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin_no);



/******************************************************************************/
/*                                                                            */
/*                     2. BUTTON API Function Prototypes                      */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Sets the Button as interrupt  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of Button
  * @retval None
  */
void button_interrupt_init(GPIO_TypeDef *GPIOx, uint16_t pin_no);


/**
  * @brief  Sets the Button as interrupt for USER_BUTTON
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of Button (Deafult Value)
  * @retval None
  */
void button_init(GPIO_TypeDef *GPIOx, uint16_t pin_no);


/**
  * @brief  Reads the state of the USER Button
  * @param  *GPIOx  : Base address of the GPIO Port
  * @param  pin_no  : pin number of Button (Deafult Value)
  * @retval uint8_t : Read value
  */
uint8_t button_read(GPIO_TypeDef *GPIOx, uint16_t pin_no);


#endif

/**************************************************** END OF FILE *************************************************/
