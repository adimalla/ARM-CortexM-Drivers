/**
  ************************************************************************
  * @file       led_m407xx.c
  * @author     Aditya Mall
  * @date       2018-07-03
  * @copyright  (C) 2018 Aditya Mall.
  * @brief      LED and USER BUTTON, Board Support Package Source file, 
  *             for ST-Discovery M407G. 
  *
  *             This file contains:
  *              - LED and BUTTON API Function implementation
  *              - Function prototypes for LED and USER BUTTON API. 
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
  * For any queries and comments mail me:-
  * @email <aditya.mall1990@gmail.com>
  *
  ************************************************************************
  */



/* Header file for LED and Button board support */
#include "led_m407xx.h"


/******************************************************************************/
/*                                                                            */
/*                     1. Led API Functions                                   */
/*                                                                            */
/******************************************************************************/


/**
  * @brief  Initialize the LEDs 
  * @param  None
  * @retval None
  */
void led_init()
{
    pin_config_t led_pin_conf;
    
    /* Enabling clock for PORT D */
    _HAL_RCC_GPIOD_CLK_ENABLE();

    /* GPIO Register configurations */
    led_pin_conf.MODE = MODER_OUTPUT;
    led_pin_conf.OPTYPE = OTYPER_PUSHPULL;
    led_pin_conf.SPEED = OSPEEDR_LOW;
    led_pin_conf.PUPD = PUPDR_NO_UP_DOWN;
    
    /* Setting pin from Green LED   */
    led_pin_conf.PIN = LED_GREEN;
    _hal_gpio_init(PORT_D, &led_pin_conf);
    
    /* Setting pin for Orange LED   */
    led_pin_conf.PIN = LED_ORANGE;
    _hal_gpio_init(PORT_D, &led_pin_conf);
    
    /* Setting pin for Red LED      */
    led_pin_conf.PIN = LED_RED;
    _hal_gpio_init(PORT_D, &led_pin_conf);
    
    /* Setting pin for BLUE LED     */
    led_pin_conf.PIN = LED_BLUE;
    _hal_gpio_init(PORT_D, &led_pin_conf);
    
}


/**
  * @brief  Turns ON the led which is connected on the given pin  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of the LED
  * @retval None
  */
void led_on(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
    _hal_gpio_write_to_pin(GPIOx, pin_no, 1);
}


/**
  * @brief  Turns OFF the led which is connected on the given pin  
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of the LED
  * @retval None
  */
void led_off(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
    _hal_gpio_write_to_pin(GPIOx, pin_no, 0);
}


/******************************************************************************/
/*                                                                            */
/*                     2. BUTTON API Functions                                */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Sets the Button as interrupt for USER_BUTTON
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of Button
  * @retval None
  */
void button_interrupt_init(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
    pin_config_t button_pin_conf;
    uint32_t exticr_button_port;
    
    /**
      * @brief Enabling clock for GPIO port A (USER_BUTTON),
      *        defining coresponding pin value for EXTICR register. 
      */
    if(GPIOx == PORT_A) {
        
        /* Enabling clock for GPIO PORT A  */
    _HAL_RCC_GPIOA_CLK_ENABLE();
        
      /* Button Port for EXTICR register */
        exticr_button_port = PA;
        
    }
    else {
        
        ; //Default, Do Nothing.
    }
        
    /* Configure GPIO Register     */
    button_pin_conf.MODE   = MODER_INPUT;
    button_pin_conf.OPTYPE = OTYPER_PUSHPULL;
    button_pin_conf.SPEED  = OSPEEDR_LOW;
    button_pin_conf.PUPD   = PUPDR_NO_UP_DOWN;
    
    /* Setting User Button pin      */
    button_pin_conf.PIN = pin_no;
    _hal_gpio_init(PORT_A, &button_pin_conf);
    
    /* Confiure Button as interupt  */
    _hal_gpio_configure_interrupt(pin_no, exticr_button_port, INT_RISING_EDGE);

    /* Enable Button EXTI interrupt */
    _hal_gpio_enable_interrupt(pin_no, EXTI0_IRQn);

}


/**
  * @brief  Sets the Button as interrupt for USER_BUTTON
  * @param  *GPIOx : Base address of the GPIO Port
  * @param  pin_no : pin number of Button (Deafult Value)
  * @retval None
  */
void button_init(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
  pin_config_t button_pin_conf;
    
    if(GPIOx == PORT_A) {
        
    /* Enabling clock for PORT A */
        _HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else {
        
        ; //Default, Do Nothing.
    }
        
    /* GPIO Register configurations */
    button_pin_conf.MODE   = MODER_INPUT;
    button_pin_conf.OPTYPE = OTYPER_PUSHPULL;
    button_pin_conf.SPEED  = OSPEEDR_LOW;
    button_pin_conf.PUPD   = PUPDR_NO_UP_DOWN;
    
    /* Setting User Button pin      */
    button_pin_conf.PIN = pin_no;
    _hal_gpio_init(PORT_A, &button_pin_conf);
    
}

/**
  * @brief  Reads the state of the USER Button
  * @param  *GPIOx  : Base address of the GPIO Port
  * @param  pin_no  : pin number of Button (Deafult Value)
  * @retval uint8_t : Read value
  */
uint8_t button_read(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
    uint8_t read_driver;
    uint8_t button_val;
    
    read_driver = 0;
    button_val = 0;
    
    read_driver = _hal_gpio_read_from_pin(GPIOx, pin_no);
    
    
    if(read_driver) {   
        
        button_val = 1; 
    }
    else {
    
        button_val = 0;
    }
    
    return button_val;
}

/**************************************************** END OF FILE *************************************************/




