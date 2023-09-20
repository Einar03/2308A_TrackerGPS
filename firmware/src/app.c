/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32Debounce.h"
#include "Mc32gest_UART.h"
#include "ADC_Driver.h"
#include "EA_DOGS164A.h"
#include "Mc32_SPI_StateMachine.h"

//#include "lcd_spi.h"
//#include "Mc32Delays.h"
//#include "Mc32_spi_sm.h"
//#include "stdio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;


static S_SwitchDescriptor DescriptBtnMode;
static S_SwitchDescriptor DescriptBtnGPS;
static S_SwitchDescriptor DescriptBtnStartStop;

DRV_HANDLE lcdHandle;
DRV_HANDLE lcdBuffer;

//uintptr_t lcdHandle;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.tracker_State = IDLE;
    // Eteindre toutes les LEDs
    GPS_ROn();
    GPS_GOff();
    GPS_BOn();
    
    SDCard_ROn();
    SDCard_GOn();
    SDCard_BOn();
    
    BatLvl_ROn();
    BatLvl_GOn();
    BatLvl_BOn();
    
//    GPS_ROff();
//    GPS_GOff();
//    GPS_BOff();
//    
//    SDCard_ROff();
//    SDCard_GOff();
//    SDCard_BOff();
//    
//    BatLvl_ROff();
//    BatLvl_GOff();
//    BatLvl_BOff();
    
//    BuzzerOff();
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    // Variables locales
    //==============================
    // Buffer pour les données réçues par le récepteur GNSS
    static uint8_t GnssData[90];
    // Compteur pour boucle for
//    uint8_t i = 0;
    // Compteur pour le temps d'envoie des données
    static uint8_t Cnt = 0;
    uint8_t dataSize = 0;
    // Variable pour le niveau de charge de la batterie
    float V_Bat = 0;
    static bool V_Bat_Mes = false;
//    static uint8_t V_Bat_Mes_Cnt = 0;
    
    // Variable pour l'intervalle de temps valeur en [s]
    uint16_t timeSpan = 30;
    uint16_t timeSpanCnt = 0;
    uint8_t timeSpanDigit = 0;
    uint16_t timeSpanIncrDecrValue;
    uint16_t timeSpanMinVal;
    uint16_t timeSpanMaxVal;
    
    // structure pour sauvegarder la valeur des régistres utilisé pour l'écran LCD
    s_EADOGS_REG_VAL RegVal;
    RegVal.displayControl = DISPLAY_CONTROL;
    RegVal.extendedFunctionSet = EXTENDED_FUNCTION_SET;
    RegVal.functionSet_RE0 = FUNCTIONSET_RE0;
    RegVal.functionSet_RE1 = FUNCTIONSET_RE1;
    RegVal.power_Display_Contrast = POWER_DISPLAY_CONTRAST;
    
    // Flags
    static bool startPointSet = false;
    static bool endPointSet = false;
    static bool USB_Connected = false;
    
    static uint8_t btnModeTime = 0;
    static uint8_t btnGPSTime = 0;
    static uint8_t btnStartStopTime = 0;
    static uint8_t btnModeCnt = 0;
    static uint8_t btnGPSCnt = 0;
    static uint8_t btnStartStopCnt = 0;
    
    static uint8_t blinkCnt = 0;
    
    
//    SPI_DoTasks();
    
    
    /* Check the application's current state. */
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // Démarrage des timers 
            DRV_TMR0_Start();  // Pour le programme principale
            DRV_TMR1_Start();  // Pour les modules OCs
            // Démarrage des OCs
            DRV_OC0_Start();   // Pour le backlight de l'écran LCD
//            DRV_OC1_Start();   // Pour le Buzzer 
            // Initialisation du ADC
            InitADC10();
            
//            V_Bat = ReadAllAnalogValues();
            // Initialisation des FIFOS pour les données reçus par le récepteur GPS
            // et les données à envoyer au PC
            InitFifoComm();
            
            // Initialisation de l'écran LCD
//            if(SPI_GetState() == SPI_STATE_IDLE)
//            {
                LCD_EADOGS_Init(&RegVal, BUS_8BITS, 4, BOTTOM_VIEW, false, false);
//            }
//            do
//            {
//                lcdHandle = DRV_SPI_Open(DRV_SPI_INDEX_0, DRV_IO_INTENT_WRITE);
//            }while(lcdHandle == DRV_HANDLE_INVALID);
//            
//            lcdBuffer = DRV_SPI_BufferAddWrite(lcdHandle, );
            
            // Initialisation des antirebonds sur les boutons 
            DebounceInit(&DescriptBtnMode);
            DebounceInit(&DescriptBtnGPS);
            DebounceInit(&DescriptBtnStartStop);
            ResetBuffer(GnssData);
            
            
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }
        case APP_STATE_WAIT:
        {
            // Rien faire
            break;
        }
        // Execution tous les 10 ms
        case APP_STATE_SERVICE_TASKS:
        {
            // Antirebonds
            // ====================
            // Bouton mode config
            DoDebounce(&DescriptBtnMode, BtnModeStateGet());
            // Bouton GPS
            DoDebounce(&DescriptBtnGPS, BtnGPSStateGet());
            // Bouton Start / Stop
            DoDebounce(&DescriptBtnStartStop, BtnStartStopStateGet());
            
            // Gestion du nombre d'appuis sur les boutons
            btnModeTime = (btnModeTime+1) % TIMEBTN;
            btnGPSTime = (btnGPSTime+1) % TIMEBTN;
            btnStartStopTime = (btnStartStopTime+1) % TIMEBTN;
            blinkCnt = (blinkCnt + 1) %  BLINK_PER;
                    
            // Si appui sur le bouton Mode config
            if(DebounceIsPressed(&DescriptBtnMode))
            {             
                // ===============================
                btnModeTime = 0;
                if(btnModeCnt < 2)
                {
                    btnModeCnt++;
                }
                //Effacer l'indicateur d'appuie car possibilité d'un appuie continu
                DebounceClearPressed(&DescriptBtnMode);
            }
            // Si appui sur le bouton GPS
            if(DebounceIsPressed(&DescriptBtnGPS))
            {             
                // ===============================
                btnGPSTime = 0;
                if(btnGPSCnt < 2)
                {
                    btnGPSCnt++;
                }
                //Effacer l'indicateur d'appuie car possibilité d'un appuie continu
                DebounceClearPressed(&DescriptBtnGPS);
            }
            // Si appui sur le bouton Start Stop
            if(DebounceIsPressed(&DescriptBtnStartStop))
            {             
                // ===============================
                btnStartStopTime = 0;
                if(btnStartStopCnt < 2)
                {
                    btnStartStopCnt++;
                }
                //Effacer l'indicateur d'appuie car possibilité d'un appuie continu
                DebounceClearPressed(&DescriptBtnStartStop);
            }
            
            // Mesure du niveau de la batterie tous les 100 ms
            if(V_Bat_Mes == false)
            {
                EN_ReadBATOn();
                V_Bat_Mes = true;
            }
            else
            {
                V_Bat = ReadAllAnalogValues();
                V_Bat_Mes = false;
                EN_ReadBATOff();
            }
            // Affichage du niveau de batterie
            // Pendant la charge
            if(V_Bat == BAT_CHARGING)
            {
                BatLvl_ROff();
                BatLvl_GOff();
                BatLvl_BOff();
            }
            // Entre 100% et 90%
            else if((V_Bat <= BAT_100) && (V_Bat >= BAT_90))
            {
                BatLvl_ROn();
                BatLvl_GOff();
                BatLvl_BOn();
            }
            // Entre 89% et 50%
            else if(V_Bat >= BAT_50)
            {
                BatLvl_ROff();
                BatLvl_GOn();
                BatLvl_BOff();
            }
            // Entre 49% et 30%
            else if(V_Bat >= BAT_30)
            {
                BatLvl_ROff();
                BatLvl_GOff();
                BatLvl_BOn();
            }
            // Entre 29% et 10%
            else if(V_Bat >= BAT_10)
            {
                BatLvl_ROff();
                BatLvl_GOn();
                BatLvl_BOn();
            }
            // Entre 9% et 0%
            else if(V_Bat < BAT_10)
            {   
                if(blinkCnt == (BLINK_PER / 2))
                {
                    BatLvl_ROff();
                }
                else if(blinkCnt == BLINK_PER)
                {
                    BatLvl_ROn();
                }
                BatLvl_GOff();
                BatLvl_BOff();
            }
            
            switch(appData.tracker_State)
            {
                // Idle
                //=====================================================
                case IDLE:
//                    GPS_ROff();
//                    GPS_GOff();
//                    GPS_BOff();
//
//                    SDCard_ROff();
//                    SDCard_GOff();
//                    SDCard_BOff();
//
//                    BatLvl_ROff();
//                    BatLvl_GOff();
//                    BatLvl_BOff();
                    // Si appui 1 fois sur le bouton GPS
                    if(btnGPSCnt == 1);
                    {
                        btnGPSCnt = 0;
                        // Passer dans l'état d'attente de connexion aux satellites
//                        appData.tracker_State = WAIT_FOR_CONNECTION;
                    }
                    if(USB_Connected == true)
                    {
                        appData.tracker_State = USB_IDLE;
                    }
                   
//                    if(SPI_GetState() == SPI_STATE_IDLE)
//                    {
//                        LCD_Printf("2308A Tracker GPS");
//                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
//                        LCD_Printf("Einar Farinas");
//                        appData.tracker_State = WAIT_FOR_CONNECTION;
//                    }
                    
                    Cnt = (Cnt + 1) % 100;
                    if(Cnt == 0)
                    {
                        ResetBuffer(GnssData);
                        dataSize = GetGnssCmd(GnssData);
                        if(dataSize != 0)
                        {
                            SendUsbMessage(GnssData, dataSize);
                        }
                    }
                    
                    
                    // Si le bouton Mode est appuyée pendant 3s
                    break;
                // Attente de connection aux satellites
                //=====================================================
                case WAIT_FOR_CONNECTION:
                    // Attendre la réecpetion d'une cordonnée
                   
                    
                    // clignotement LED GPS
                    
                    break;
                // Connexion aux satellites réussie
                // système prêt à acquerir et sauvegarder des coordonnées
                //=====================================================
                case READY:
                    // Si appui 1 fois sur le bouton GPS et le point de départ n'a pas été sauvegardé
                    if((btnGPSCnt == 1) && (startPointSet == false))
                    {
                        // Retour à l'état idle
                        appData.tracker_State = IDLE;
                        btnGPSCnt = 0;
//                        break;
                    }
                    // Si appui 1 fois sur le bouton GPS et et point de départ a été sauvegardé
                    else if((btnGPSCnt == 1) && (startPointSet == true))
                    {
                        // Retour à l'état run pour l'acquisition des coordonnées
                        appData.tracker_State = RUN;
                        // Reset flag
                        startPointSet = false;
                        btnGPSCnt = 0;
//                        break;
                    }
                    // Si appui sur le bouton Start
                    else if((btnStartStopCnt == 1) && (startPointSet == false))
                    {
                        // Set le flag du point de départ
                        startPointSet = true;
                        btnStartStopCnt = 0;
                        // Savegarde la coordonnée de départ 
                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    break;
                // Acquisition et savegarde de coordonnées selon l'intervalle de temps
                //===================================================== 
                case RUN:
                    
                    // Si temps d'intervalle écoulé
                    if(timeSpanCnt == timeSpan)
                    {
                        // Reset compteur
                        timeSpanCnt = 0;
                        // Savegarde la coordonnée de départ 
                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    // Si appui une première fois sur le bouton start/stop
                    // Set flag pour indiquer le point d'arrivé
                    if((btnStartStopCnt == 1)  && (endPointSet == false))
                    {
                        // Set flag du point d'arrivée
                        endPointSet = true;
                        // Reset compteur
                        btnStartStopCnt = 0;
                    }
                    // Si flag de point d'arrivé set et appui 2 fois sur 
                    // bouton GPS
                    // Confirmer le point d'arrivé
                    else if((btnStartStopCnt == 2) && (endPointSet == true))
                    {
                        // Savegarde la coordonnée de départ 
                        appData.tracker_State = SAVE_COORDINATES;
                        // Reset compteur
                        btnStartStopCnt = 0;
                    }
                    else if(btnStartStopCnt != 0)
                    {
                        // Reset flag du point d'arrivée
                        endPointSet = false;
                        // Reset compteur
                        btnStartStopCnt = 0;
                    }
                    
                    
//                    
                    
                    break;
                // Savegarde coordonnées dans la carte SD
                //=====================================================
                case SAVE_COORDINATES:
                    // Si sauvegarde du point d'arrvié ou de départ
                    if((startPointSet == true) || (endPointSet == true))
                    {
                        // Reset flag d'arrivé
                        endPointSet = false;
                        // Passer dans l'état READY
                        appData.tracker_State = READY;
                    }
                    // Si sauvegarde du point selon intervalle de temps
                    else if((startPointSet == false) && (endPointSet == false))
                    {
                        // Retourner dans l'état run
                        appData.tracker_State = RUN;
                    }
                    break;
                //=====================================================
                // Lors de la connection par USB au PC
                //=====================================================
                // Attente de demande de lecture des données sur la carte SD
                case USB_IDLE:
                    break;
                // Lecture et récupération des données sur la carte SD
                case USB_READ_DATA:
                    break;
                // Envoi des données récupérées par USB au PC
                case USB_SEND_DATA:
                    break;
                // Mode configuration 
                // Pour régler l'intervalle de temps ou autres paramètres
                case CONFIG:
                    
                    break;
                // Pour sauvegarder la configuration choisie
                case SAVE_CONFIG:
                    // Selection du digit
                    //======================
                    if(btnModeCnt == 1)
                    {
                        // Deplacement du curseur au digit suivante
                        btnModeCnt = 0;
                    }
                    else if(btnModeCnt != 0)
                    {
                        btnModeCnt = 0;
                    }
                    switch(timeSpanDigit)
                    {
                        // Secondes unitées
                        case 0:
                            timeSpanIncrDecrValue = 1;
                            break;
                        // Secondes dixièmes
                        case 1:
                            timeSpanIncrDecrValue = 10;
                            break;
                        // Minutes unitées
                        case 2:
                            timeSpanIncrDecrValue = 60;
                            break;
                        // Minutes dixièmes
                        case 3:
                            timeSpanIncrDecrValue = 600;
                            break;
                        default:
                            break;
                    }
                    // Set les valeurs min et max pour pouvoir
                    // incrémeter ou décrémenter la valeur
                    timeSpanMinVal = timeSpanIncrDecrValue;
                    timeSpanMaxVal = (TIME_SPAN_MAX - timeSpanIncrDecrValue);
                    
                    // Incrémentation de la valeur du digit
                    // avec le bouton StartStop
                    if(btnStartStopCnt == 1)
                    {
                        if(timeSpan <= timeSpanMaxVal)
                        {
                            timeSpan += timeSpanIncrDecrValue;
                        }
                        btnStartStopCnt = 0;
                    }
                    else if(btnStartStopCnt != 0)
                    {
                        btnStartStopCnt = 0;
                    }
                    // Décrémentation de la valeur du digit
                    // avec le boutons GPS
                    if(btnGPSCnt == 1)
                    {
                        if(timeSpan >= timeSpanMinVal)
                        {
                            timeSpan -= timeSpanIncrDecrValue;
                        }
                        btnGPSCnt = 0;
                    }
                    else if(btnGPSCnt != 0)
                    {
                        btnGPSCnt = 0;
                    }
                    
                    break;
                default:
                    break;
                    
            }
//            
//          
            
//            GetGnssMessage(GnssData, 50);
//            
//            
//            
//            Cnt = (Cnt + 1) % 50;
//            if(Cnt == 0)
//            {
//                SendUsbMessage(GnssData, 70);
//                EN_ReadBATOn();
//                V_Bat = ReadAllAnalogValues();
//                EN_ReadBATOff();
//                if(SPI_GetState() == SPI_STATE_IDLE)
//                {
//                    LCD_Printf("2308A Tracker GPS ");
//                }
//                SendUsbMessage(GnssData, 30);
//            }
            
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void APP_UpdateState(APP_STATES newState)
{
    appData.state = newState; // mise à jour d'état
}

void Timer1_CallBack(void)
{
    // Compteur pour pour le changment de LEDS
    static uint8_t Cnt = 0;
    static uint8_t SelectLed = 0;
    
    
    
    // Comtpteur de 0 à 100 pour chager de LED chaque 1 seconde (T timer = 10 ms) 
    Cnt = (Cnt + 1) % CNT_TIME;
    
    if(Cnt == 0)
    {
        SelectLed = (SelectLed + 1) % NB_ACTIONS;
        switch(SelectLed)
        {
            case 0:
                GPS_ROff();
                BatLvl_BOn();
                break;
            case 1:
                GPS_ROn();
                GPS_GOff();
                break;
            case 2:
                GPS_GOn();
                GPS_BOff();
                break;
            case 3:
                GPS_BOn();
                SDCard_ROff();
                break;
            case 4:
                SDCard_ROn();
                SDCard_GOff();
                break;
            case 5:
                SDCard_GOn();
                SDCard_BOff();
                break;
            case 6:
                SDCard_BOn();
                BatLvl_ROff();
                break;
            case 7:
                BatLvl_ROn();
                BatLvl_GOff();
                break;
            case 8:
                BatLvl_GOn();
                BatLvl_BOff();
                break;
            default:
                break;
        }
    }
    
    
}

void ResetBuffer(uint8_t *Buffer)
{
    uint8_t i;
    
    for(i=0; i<90;i++)
    {
        Buffer[i] = 0;
    }
}

APP_STATES AppGetState(void)
{
    return appData.state;
}
/*******************************************************************************
 End of File
 */
