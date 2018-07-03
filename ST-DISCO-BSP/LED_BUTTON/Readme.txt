  **************************************************************************
  * File       led_m407xx.h
  * Author     Aditya Mall
  * Date       2018-07-03
  * copyright  (C) 2018 Aditya Mall.
  * Brief      LED and USER BUTTON, Board Support Package Header file, 
  *             for ST-Discovery M407G. 
  *
  ***************************************************************************
  * Attention:
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
 
 
  *****************************************************************************
  * 
  *                     !! HOW TO USE THIS PACKAGE !!
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
  * 
  ********************************************************************************
    
  
  *********************************************************************************
  * Version 1.0 - First release
  * Brief   LED and Button Board support package interface for STM-Discovery-F407,
  *          by ST Microelectronics
  *
  * Dependencies gpio_driver_m407xx.h  (Aditya Mall)
  *               gpio_driver_m407xx.c  (Aditya Mall) 
  *               system_stm32f4xx.c    (Vendor Specific) 
  *               startup_stm32f407xx.s (Vendor Specific)
  *
  **********************************************************************************
