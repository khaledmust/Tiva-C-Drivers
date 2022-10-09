/*
 * gpio_driver.h
 *
 *  Created on: Sep 5, 2022
 *      Author: Khaled Mustafa
 */

#ifndef DRIVERS_INC_GPIO_DRIVER_H_
#define DRIVERS_INC_GPIO_DRIVER_H_

#include "tm4c123xxx.h"

/**< Enables or Disables the Advanced High Peripheral Bus. */
#define GPIO_AHP_BUS_STATUS ENABLE

#define PIN_HIGH	0xFF
#define PIN_LOW		0x00

#define PORT_HIGH	0xFF
#define PORT_LOW	0x00

#define GPIO_UNLOCK 0x4C4F434B
#define GPIO_LOCK 0x00000000

/************************************************
 *  @struct GPIO_PinConfig_t
 *  @brief GPIO peripheral configuration structure.
 *  @warning In case of output pin an attachment state must be defined, else
 *  		the Open Drain configuration will be chosen by default and undefined error results.
 *  @bug If no attachment state the default is ODC.
 ***********************************************/
typedef struct
{
	uint8_t GPIO_PinNum;				/**< Select pin number to be configured. */
	uint8_t GPIO_PinDirection;			/**< Select the direction of the pin to be configured. */
	uint8_t GPIO_AlternateFunction;		/**< Select alternate function for the pin. */
	uint8_t GPIO_CurrentStrenght;		/**< Select drive current strength for the selected pin. */
	uint8_t GPIO_PinType;				/**< Selects the attachment type for the pin. */
	uint8_t GPIO_AnalogCtrl;			/**< Enable the analog functionality of the selected pin. */
}GPIO_PinConfig_t;

/************************************************
 *  GPIO peripheral handle.
 *  This the only interface for the user to access the GPIO settings.
 ***********************************************/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/**< Address of the GPIO register to be configured. */
	GPIO_PinConfig_t *pGPIO_PinConfig;	/**< Address of the configuration structure holding the pin configuration. */
}GPIO_Handle_t;

///************************************************
// *  @enum GPIO_PinNum_t
// *  8 values corresponding to the number of pins for each port.
// ***********************************************/
//typedef enum
//{
//	GPIO_PIN0,
//	GPIO_PIN1,
//	GPIO_PIN2,
//	GPIO_PIN3,
//	GPIO_PIN4,
//	GPIO_PIN5,
//	GPIO_PIN6,
//	GPIO_PIN7
//}GPIO_PinNum_t;

#define GPIO_PIN_0              0x00000001  // GPIO pin 0
#define GPIO_PIN_1              0x00000002  // GPIO pin 1
#define GPIO_PIN_2              0x00000004  // GPIO pin 2
#define GPIO_PIN_3              0x00000008  // GPIO pin 3
#define GPIO_PIN_4              0x00000010  // GPIO pin 4
#define GPIO_PIN_5              0x00000020  // GPIO pin 5
#define GPIO_PIN_6              0x00000040  // GPIO pin 6
#define GPIO_PIN_7              0x00000080  // GPIO pin 7

/************************************************
 *  @enum GPIO_Direction_t
 *  Two values the sets the data direction of GPIO pin.
 ***********************************************/
typedef enum
{
	GPIO_DIR_INPUT = 0,
	GPIO_DIR_OUTPUT
}GPIO_Direction_t;

/************************************************
 *  @enum GPIO_InternalAttach_t
 *  Three values that sets the internal attachment type of a GPIO pin.
 ***********************************************/
typedef enum
{
	GPIO_ODR = 0,
	GPIO_PU,
	GPIO_PD
}GPIO_InternalAttach_t;

/************************************************
 *  @enum GPIO_DriveStrength_t
 *  Three values that sets the current strength level of a GPIO pin.
 ***********************************************/
typedef enum
{
	GPIO_DR2R = 0,
	GPIO_DR4R,
	GPIO_DR84
}GPIO_DriveStrength_t;

/************************************************
 *  @enum GPIO_Digital_t
 *  Two values that sets the state of digital functionality of a GPIO pin.
 ***********************************************/
typedef enum
{
	GPIO_DIG_DSBL = 0,
	GPIO_DIG_ENB
}GPIO_Digital_t;



/************************************************
 *  @brief Enables or disables the clock to the selected GPIO port.
 *  Enable the clock to the port by setting the appropriate bits in the RCC GPIO register
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param ClkState The state of the clock either enable or disable.
 *  @return None
 ***********************************************/
void GPIO_ClockCtrl (GPIO_RegDef_t *pGPIOx, uint8_t ClkState);

//static void GPIO_AHB_ClockCtrl (GPIO_RegDef_t *pGPIOx, uint8_t ClkState);

/************************************************
 *  @brief Initialize the GPIO pin with the user configuration.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured and user configurations.
 *  @return None
 ***********************************************/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle);

/************************************************
 *  @brief De-initialize the GPIO pin.
 *  De-initialization is done by setting the value of a 6-bit register corresponding to the required port.
 *  @see Page 314 of the Tiva-C launchpad documentation.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @return None
 ***********************************************/
void GPIO_DeInit (GPIO_Handle_t *pGPIOHandle);

/************************************************
 *  @brief Read the value of a bit corresponding to the pin selected.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PinNum Pin number to be read from.
 *  @return Value of the pin read.
 ***********************************************/
uint8_t GPIO_ReadPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/************************************************
 *  @brief Read a value of a 8-bit corresponding to the port selected.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @return Value of the port read.
 ***********************************************/
uint8_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx);

/************************************************
 *  @brief Write the binary value in the data register to the pin selected.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PinNum Pin number to be written to.
 *  @param PinValue Value to be written on the pin.
 *  @return None
 ***********************************************/
void GPIO_WritePin (GPIO_Handle_t *pGPIOHandle, uint8_t PinValue);

/************************************************
 *  @brief Write an 8-bit value in the data register of the selected port.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PortValue 8-bit value to be written on the whole port.
 *  @return None
 ***********************************************/
void GPIO_WritePort (GPIO_Handle_t *pGPIOHandle, uint8_t PortValue);

/************************************************
 *  @brief Toggle the state of a pin.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PinNum Pin number to be toggled.
 *  @return None
 ***********************************************/
void GPIO_TogglePin (GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

static void GPIO_Lock (GPIO_Handle_t *pGPIOHandle);
static void GPIO_Unlock (GPIO_Handle_t *pGPIOHandle);

#endif /* DRIVERS_INC_GPIO_DRIVER_H_ */
