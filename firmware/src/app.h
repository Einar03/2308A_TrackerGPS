/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "Mc32Debounce.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define CNT_TIME        100
#define NB_ACTIONS        9
    
// Boutons
#define TIMEBTN          30
#define NB_PRESS_BTN_MAX  2
#define BL_100         4999
// Intervalle d'enregistrement
#define TIME_SPAN_MAX   900 // pour 15 min (900)
#define TIME_SPAN_MIN    30 // pour 15 min (900) 
#define BLINK_PER        30 // Pour 10 Hz (10 == 100 ms)
// Pour incrémentation/décrémentation de l'intervalle de temps ou le niveau du backlight
// lors qu'on mantient appuyée sur le bouton
#define INCREMENT_HOLD_SPAN   7
// Pour le temps minimum requis de mantient sur un bouton, pour incrémenter/décrémenter
// l'intervalle de temps ou le niveau du backlight en continu 
#define HOLD_MIN_TIME    100
// Niveau de batterie
#define BAT_100         4.20  
#define BAT_90          4.08
#define BAT_50          3.60
#define BAT_30          3.36
#define BAT_10          3.12
#define BAT_0           3.00
#define BAT_TIME         300      // 300 pour 3s 
#define LED_PER            2
// LCD
#define BACKLIGHT_LVL_MAX 100
#define BACKLIGHT_LVL_MIN   0
// USB
#define USB_TIME_OUT     200
    
    
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_SERVICE_TASKS,
    APP_STATE_WAIT,
    APP_STATE_OFF,
            
	/* TODO: Define states used by the application state machine. */

} APP_STATES;

typedef enum
{
    INIT = 0,
	IDLE,
	WAIT_FOR_CONNECTION,
    READY,
    TRACKING,
    SAVE_COORDINATES,
    USB_IDLE,
    USB_READ_DATA,
    USB_SEND_DATA,
    CONFIG,
    SAVE_CONFIG,
} PROGRAM_STATES;

typedef enum
{
    START_MENU_1 = 0,
    START_MENU_2,
    TIME_SPAN,
    BACKLIGHT,
            
} MENU_STATES;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    PROGRAM_STATES tracker_State;
    /* TODO: Define any additional data used by the application. */
} APP_DATA;

typedef struct
{
    uint8_t  direction;
    uint16_t degrees;
	uint8_t  min;
	float    sec;
} s_CoordinatesParse;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

void APP_UpdateState (APP_STATES newState);
void Timer1_CallBack(void);
void ResetBuffer(uint8_t *Buffer);
APP_STATES AppGetState(void);
void ScanButtons(S_SwitchDescriptor *DescriptButton, bool ButtonState);
void ReadBattery(float *VBat);
bool BuzzerPlay(uint16_t duration, uint8_t note);

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

