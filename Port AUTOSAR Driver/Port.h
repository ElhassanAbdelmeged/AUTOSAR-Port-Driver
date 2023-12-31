 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Elhassasn Abdelmeged Mohamed
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H


/*******************************************************************************
 *                              definitions and important macros                          *
 *******************************************************************************/

/* Id for the company in the AUTOSAR
 *  for example Mohamed Tarek's ID = 1000 */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*Port Status*/
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)



/*******************************************************************************
 *         AUTOSAR Version checking between Std Types and Port Modules         *
 *******************************************************************************/


/* including Standard AUTOSAR types */
#include "Std_Types.h"


#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif



/*******************************************************************************
 *          AUTOSAR&SW Version checking between Port_Cfg.h and Port.h files       *
 *******************************************************************************/

/* including Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Dio_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Initializes the Port Driver module.  */
#define PORT_INIT_SID                  (uint8)0x00

/* Service ID for Sets the port pin direction  */
#define PORT_SET_PIN_DIRECTION_SID       (uint8)0x01

/* Service ID for Refreshes port direction.  */
#define PORT_REFRESH_PORT_DIRECTION_SID  (uint8)0x02

/* Returns the version information of this module */
#define PORT_GET_VERSION_INFO_SID        (uint8)0x03

/* Service ID for Sets the port pin mode */
#define Port_SET_PIN_MODE_SID     (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN               (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE  (uint8)0x0B

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG            (uint8)0x0C

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE      (uint8)0x0D

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E

/* API service called without module initialization */
#define PORT_E_UNINIT                   (uint8)0x0F
   
/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER            (uint8)0x10

/*******************************************************************************
 *                      GPIO Alternative Functions (Modes)                     *
 ******************************************************************************/
#define DIO                             (Port_PinModeType)0
#define UART                            (Port_PinModeType)1
#define SSI                             (Port_PinModeType)2
#define I2C                             (Port_PinModeType)3
#define PWM4                            (Port_PinModeType)4
#define PWM5                            (Port_PinModeType)5
#define IDX                             (Port_PinModeType)6
#define TIMER                           (Port_PinModeType)7
#define CAN                             (Port_PinModeType)8
#define TRD                             (Port_PinModeType)14
/* Special code to identify the Analog mode */
#define ANALOG                          (Port_PinModeType)10
   




/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/


/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Type definition for Port_PinNum used by the PORT APIs */
typedef uint8 Port_PortType;

/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT,OUTPUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistorType;

/* Type definition for Port_PortModeType used by the PORT APIs */
typedef uint8 Port_PinModeType;

/* Description: Enum to hold changing modes or direction for PIN */
typedef enum
{
    DISABLLED, ENABLLED
}Port_Changeability;

/* Description: Structure to configure each individual PIN:
 *  1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *  2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *  5. the mode which the pin support.
 *  6. the pin initial value in case of I/O pin.
 *  7. the abillity to change the pin mode.
 *  8. the abillity to change the pin direction.
 */
typedef struct 
{
    Port_PortType Port_Num;
    Port_PinType Pin_Num; 
    Port_PinDirectionType direction;
    Port_InternalResistorType resistor;
    Port_PinModeType Pin_mode;
    uint8 initial_value;
    Port_Changeability Port_IsModeChangable;
    Port_Changeability Pin_IsDirectionChangable;
}Port_ConfigPin;

/* Structure required for initializing the Port Driver */
typedef struct Port_ConfigType
{
	Port_ConfigPin Pins[PORT_CONFIGURED_PINS];
}Port_ConfigType;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
*Service Name: Port_Init
* Service ID: 0x00
*Sync/Async: Synchronous
*Reentrancy: Non-Reentrant
*Parameters (in): ConfigPtr - Pointer to post-build configuration data
*Parameters (inout): None
*Parameters (out): None
*Return value: None
*Description: Initializes the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr);


/************************************************************************************
*Service Name: Port_SetPinDirection
*Sync/Async: Synchronous
* Service ID: 0x01
*Reentrancy: Reentrant
*Parameters (in): Pin - The Pin ID whose direction is to be changed.
*Parameters (inout): None
*Parameters (out): None
*Return value: None
*Description: Sets the direction (input or output) of a specific port pin.
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_ApI == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction); 
#endif



/************************************************************************************
*Service Name: Port_RefreshPortDirection
** Service ID: 0x02
*Sync/Async: Synchronous
*Reentrancy: Reentrant
*Parameters (in): None
*Parameters (inout): None
*Parameters (out): None
*Return value: None
*Description: Updates the direction (input or output) of all configured port pins.
************************************************************************************/
void Port_RefreshPortDirection(void);

/************************************************************************************
*Service Name: Port_GetVersionInfo
*Service ID:0x03
*Sync/Async: Synchronous
*Reentrancy: Non-Reentrant
*Parameters (in): None
*Parameters (inout): None
*Parameters (out): versioninfo - Pointer to where to store the version information of the Port module.
*Return value: None
*Description: Returns the version information of the Port module. 
 ************************************************************************************/ 
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo); 
#endif


/************************************************************************************
*Service Name: Port_SetPinMode
** Service ID: 0x04
*Sync/Async: Synchronous
*Reentrancy: Reentrant
*Parameters (in): Pin - The Pin ID whose mode is to be changed.
*Parameters (inout): None
*Parameters (out): None
*Return value: None
*Description: Sets the mode of a specific port pin.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);

void Port_SetupGpioPin(const Port_ConfigType * ConfigPtr );
/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
