/**
  ************************************************************************
  * @file       gpio_driver_m407xx.c
  * @author     Aditya Mall
  * @date       2018-06-25
  * @copyright  (C) 2018 Aditya Mall.
  * @brief      STM32F407xx GPIO Driver Implementation File
  *
  *             This file contains:
  *              - Helper functions for Driver exposed API functions
  *              - Driver exposed APIs, for GPIO
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


/* Driver header file */
#include "gpio_driver_m407xx.h"
  
  
/******************************************************************************/
/*                                                                            */
/*               1. Helper function for APIs                                  */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Set the mode register for a specific pin.
  * @param  *GPIOx   : GPIO port base address
  * @param  pin_no   : GPIO pin number
  * @param  pin_mode : Mode to be configured
  * @retval none  
  */
static void _hf_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_mode)
{
	GPIOx->MODER &= ~(0X03 << (pin_no * 2));
	GPIOx->MODER |= (pin_mode <<(pin_no * 2));	
}


/**
   * @brief  Configures the output type of a pin  
   * @param  *GPIOx     : GPIO Port Base address
   * @param  pin_no     : GPIO pin number 
   * @param  pin_optype : output type to be configured with 
   * @retval None
   */
static void _hf_gpio_configure_pin_optype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t pin_optype)
{
	GPIOx->OTYPER &= ~(1 << pin_no);
	GPIOx->OTYPER |= (pin_optype << pin_no);
}


/**
  * @brief  Configures the speed of a pin 
  * @param  *GPIOx    : GPIO Port Base address
  * @param  pin_no    : GPIO pin number 
  * @param  pin_speed : value of the speed 
  * @retval None
  */
static void _hf_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_speed)
{
	GPIOx->OSPEEDR &= ~(1 << (pin_no * 2));
	GPIOx->OSPEEDR |= (pin_speed << (pin_no * 2));
}


/**
  * @brief  Activates the internall pull up or pull down resistors
  * @param  *GPIOx   : GPIO Port Base address
  * @param  pin_no   : GPIO pin number 
  * @param  pin_pupd : specifies which resistor to activate
  * @retval None
  */
static void _hf_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pin_pupd)
{
	GPIOx->PUPDR &= ~(1 << (pin_no * 2));
	GPIOx->PUPDR |= (pin_pupd << (pin_no * 2));	
}



/******************************************************************************/
/*                                                                            */
/*               2. Driver exposed API Function                               */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Intializes GPIO pin
  * @param  *GPIOx         : GPIO port base address 
  * @param  *gpio_pin_conf : pointer to the gpio pin config structure.
  * @retval None.    
  */
void _hal_gpio_init(GPIO_TypeDef *GPIOx, pin_config_t *gpio_pin_conf)
{
	_hf_gpio_configure_pin_mode(GPIOx, gpio_pin_conf->PIN, gpio_pin_conf->MODE);	  /* This helper function configures pin mode          */
	
	_hf_gpio_configure_pin_optype(GPIOx, gpio_pin_conf->PIN, gpio_pin_conf->OPTYPE);  /* This helper function configures output type       */
	
	_hf_gpio_configure_pin_speed(GPIOx, gpio_pin_conf->PIN, gpio_pin_conf->SPEED);    /* This helper function configures peripheral speed  */
	
	_hf_gpio_configure_pin_pupd(GPIOx, gpio_pin_conf->PIN, gpio_pin_conf->PUPD);      /* This helper function configures pull up/down mode */	
}


/**
  * @brief  Reads value from the GPIO pin
  * @param  *GPIOx  : GPIO port base address 
  * @param  pin_no  : GPIO pin number
  * @retval uint8_t : Read Value
  */
uint8_t _hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
	uint8_t read_val;
	
	read_val = (GPIOx->IDR >> pin_no) & 0x01U;       /* Right shift by pin_no and mask lsb with masking bit */
	
	return read_val;
}


/**
  * @brief  Writes value to the GPIO pin
  * @param  *GPIOx : GPIO port base address 
  * @param  pin_no : GPIO pin number
  * @param  val    : if val is 1 then ODR register is set as per pin number
  * @retval None    
  */
void _hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val)
{
	if(val) {
	  if( !(GPIOx->ODR & (1 << pin_no)) ) {
		
		  GPIOx->ODR |= (1 << pin_no);
	  } 
		else {

			; // default, do nothing
		}
	}
	else {
		
		GPIOx->ODR &= ~(1 << pin_no);
	}
}


/**
  * @brief  Set the alternate functinonality to the given pin.
  * @param  *GPIOx         : GPIO port base address 
  * @param  pin_no         : GPIO pin number
  * @param  alt_func_value : value to be written
  * @retval None    
  */
void _hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t alt_func_value)
{
	if(pin_no <= 7) {
		
		/* Setting Alternate function low registers */
		GPIOx->AFR[0] &= ~(alt_func_value << (pin_no * 4));
		GPIOx->AFR[0] |= (alt_func_value << (pin_no * 4));
	
	}
	else {
		
		/* Setting ALternate function high registers */
		GPIOx->AFR[1] &= ~(alt_func_value << ((pin_no % 8) * 4));
		GPIOx->AFR[1] |= (alt_func_value << ((pin_no % 8) * 4));
		
	}
}

/**
  * @brief  Atomi bit set reset for GPIO pin
  * @param  *GPIOx  : GPIO port base address 
  * @param  pin_no  : GPIO pin number
  * @param  val     : if 1 then the ODR bit is set.
  * @retval None
  */
void _hal_gpio_bit_set_reset_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val)
{
	if(val) {
		
		GPIOx->BSRR |= (1 << pin_no);							/* Set ODRx bit   */
	}
	else {
		
		GPIOx->BSRR |= (1 << (pin_no + 16));      /* Reset ORDx bit */
	}
}


/******************************************************************************/
/*                                                                            */
/*               3. Driver exposed API Interupts Functions                    */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  Configure the interrupt for a given pin number   
  * @param  pin_no   : GPIO pin number 
  * @param  pin_port : Corresponding GPIO Port for the EXTICR register
  * @param  edge_sel : Triggering edge slection value of type "int_edge_sel_t"
  * @retval None
  */
void _hal_gpio_configure_interrupt(uint16_t pin_no, uint8_t pin_port, edge_select_t edge_sel)
{
	/* Setting different trigger modes */
	if(edge_sel == INT_RISING_EDGE) {
		
		EXTI->RTSR |= (1 << pin_no);
	}
	else if(edge_sel == INT_FALLING_EDGE) {
		
		EXTI->FTSR |= (1 << pin_no);
	}
	else if (edge_sel == INT_RISING_FALLING_EDGE) {
		
		EXTI->RTSR |= (1 << pin_no);
		EXTI->FTSR |= (1 << pin_no);
	}
	else {
		
		EXTI->RTSR &= ~(1 << pin_no);
		EXTI->FTSR &= ~(1 << pin_no);
	}
	
	/* Setting EXTI interrupt for the pins */
	if(pin_no <= 3) {
		
		SYSCFG->EXTICR[0] |= (pin_port << (pin_no * 4));
	}
	else if(pin_no > 3 && pin_no <= 7) {
		
	  SYSCFG->EXTICR[1] |= (pin_port << ((pin_no % 4) * 4));
	}
	else if(pin_no > 7 && pin_no <= 11) {
		
		SYSCFG->EXTICR[2] |= (pin_port << ((pin_no % 8) * 4));
	}
	else if(pin_no > 11 && pin_no <= 15) {
		
		SYSCFG->EXTICR[3] |= (pin_port << ((pin_no % 12) * 4));
	}
	else {
		
		; // default, do nothing.
	}
}


/**
  * @brief  Enable the interrupt for a give pin number and irq number  
  * @param  pin_no : GPIO pin number 
  * @param  irq_no : irq_number to be enabled in NVIC 
  * @retval None
  */
void _hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no)
{
	/* Enabling clock at SYSCFG for EXT interupts */
	RCC->APB2ENR |= (1 << 14);
	
	/* Setting the IMR register for EXTI interrupt */
	EXTI->IMR |= (1 << pin_no);
	NVIC_EnableIRQ(irq_no);	
}


/**
  * @brief  Clear the interrupt pending bit if set 
  * @param  pin_no : GPIO pin number 
  * @retval None
  */
void _hal_gpio_clear_interrupt(uint16_t pin_no)
{
	if(EXTI->PR & (1 << pin_no)) {
		
		EXTI->PR |= (1 << pin_no);
	}
}
/**************************************************** END OF FILE *************************************************/
