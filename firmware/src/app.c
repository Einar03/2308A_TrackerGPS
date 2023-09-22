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
#include "minmea.h"

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

//DRV_HANDLE lcdHandle;
//DRV_HANDLE lcdBuffer;

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
    appData.tracker_State = INIT;
    // Eteindre toutes les LEDs
    GPS_ROff();
    GPS_GOff();
    GPS_BOff();
    
    SDCard_ROff();
    SDCard_GOff();
    SDCard_BOff();
    
    BatLvl_ROff();
    BatLvl_GOff();
    BatLvl_BOff();
    

    
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
    //============================================================
    // Variables locales
    //============================================================
    
    //                      GNSS
    //=====================================================
    // Buffer pour les données réçues par le récepteur GNSS
    static uint8_t GnssData[90];
    
    //                 Compteurs
    //=====================================================
    // Compteur pour le temps d'envoie des données
    static uint8_t Cnt = 0;
    uint8_t dataSize = 0;
    
    //                  Batterie
    //=====================================================
    // Variable pour la tension de la batterie
    static float V_Bat = 0;
    // Flag pour activer la mesure de tension de la batterie
//    static bool V_Bat_Mes = false;
    static uint16_t V_Bat_Mes_Cnt = 0;
    
    //               Intervalle de sauvegarde
    //=====================================================
    // Consigne de l'intervalle en [s], valeur par défaut au min (30s)
    // valeur max 15 min = 900s
    uint16_t timeSpan = 30;
    // Compteur de temps pour l'intervalle en [s]
    uint16_t timeSpanCnt = 0;
    // Variable pour la position du digit à modifier de l'intervalle dans le mode config
    // Ex pour interevalle de  12:34
    // 0 => 4
    // 1 => 3
    // 2 => 2
    // 3 => 1
    uint8_t timeSpanDigit = 0;
    // Variable pour choisir la valeur à ajouter ou enlever de l'intervalle 
    // en fonction du digit choisi
    uint16_t timeSpanIncrDecrValue;
    // Variable pour la valeur minimum de l'intervalle en fonction
    // du digit sélectionné
    uint16_t timeSpanMinVal;
    // Variable pour la valeur maximum de l'intervalle en focntion 
    // du digit sélectionnné
    uint16_t timeSpanMaxVal;
    // Ex si timeSpandigit = 0 :
    // min =   1    =>    (0 + 1s)
    // max = 899    =>    (900 - 1s)
    // Ex si timeSpandigit = 1 :
    // min =  10    =>    (0 + 10s)
    // max = 890    =>    (900 - 10s)
    
    
    //                         LCD
    //=====================================================
    // structure pour sauvegarder la valeur des régistres utilisé pour l'écran LCD
    s_EADOGS_REG_VAL RegVal;
    RegVal.displayControl = DISPLAY_CONTROL;
    RegVal.extendedFunctionSet = EXTENDED_FUNCTION_SET;
    RegVal.functionSet_RE0 = FUNCTIONSET_RE0;
    RegVal.functionSet_RE1 = FUNCTIONSET_RE1;
    RegVal.power_Display_Contrast = POWER_DISPLAY_CONTRAST;
    
    //               Flags
    //=====================================================
    // Point de départ
    static bool startPointSet = false;
    // Point d'arrivé
    static bool endPointSet = false;
    bool SDCardmounted = false;
    
    //               USB
    //=====================================================
    // Connexion à l'application du PC
    static bool USB_Connected = false;
    // Buffer de réception
    static uint8_t USB_RxBuffer[20];
//    static uint8_t USB_TxBuffer[512];
    static uint8_t KeepAliveCnt = 0;
    
    
    //               Boutons
    //=====================================================
    // structures pour l'antirebond
    static S_SwitchDescriptor DescriptBtnMode;
    static S_SwitchDescriptor DescriptBtnGPS;
    static S_SwitchDescriptor DescriptBtnStartStop;
    // Compteurs pour le nombre d'appuis sur chaque bouton
//    static uint8_t btnModeTime = 0;
//    static uint8_t btnGPSTime = 0;
//    static uint8_t btnStartStopTime = 0;
//    static uint8_t btnModeCnt = 0;
//    static uint8_t btnGPSCnt = 0;
//    static uint8_t btnStartStopCnt = 0;
    
    // Compteur pour le clignotement des LEDs
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
            
//            
            // Initialisation de l'écran LCD
            if(SPI_GetState() == SPI_STATE_IDLE)
            {
                LCD_EADOGS_Init(&RegVal, BUS_8BITS, 4, BOTTOM_VIEW, false, false);
            }
           
            
            // Initialisation des antirebonds sur les boutons 
            DebounceInit(&DescriptBtnMode);
            DebounceInit(&DescriptBtnGPS);
            DebounceInit(&DescriptBtnStartStop);
            // Reset buffer de datas du récepteur GNSS
            ResetBuffer(GnssData);
            
            // Passer dans l'état d'attente
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
            //=========================
            // Execution du antirebonds
            //=========================
           
            // Bouton mode config
            ScanButtons(&DescriptBtnMode, BtnModeStateGet());
            // Bouton GPS
            ScanButtons(&DescriptBtnGPS, BtnGPSStateGet());
            // Bouton Start / Stop
            ScanButtons(&DescriptBtnStartStop, BtnStartStopStateGet());
            
//            //=================================================
//            // Gestion du nombre d'appuis sur les boutons
//            //=================================================
//            btnModeTime = (btnModeTime+1) % TIMEBTN;
//            btnGPSTime = (btnGPSTime+1) % TIMEBTN;
//            btnStartStopTime = (btnStartStopTime+1) % TIMEBTN;
//            blinkCnt = (blinkCnt + 1) %  BLINK_PER;
//                    
//            // Si appui sur le bouton Mode config
//            //=================================================
//            if(DebounceIsPressed(&DescriptBtnMode))
//            {             
//                // Reset compteur de timeout
//                btnModeTime = 0;
//                
//                if(btnModeCnt < NB_PRESS_BTN_MAX)
//                {
//                    btnModeCnt++;
//                }
//                // 
//                DebounceClearPressed(&DescriptBtnMode);
//            }
//            // Si appui sur le bouton GPS
//            //=================================================
//            if(DebounceIsPressed(&DescriptBtnGPS))
//            {             
//                // Reset compteur de timeout
//                btnGPSTime = 0;
//                if(btnGPSCnt < NB_PRESS_BTN_MAX)
//                {
//                    btnGPSCnt++;
//                }
//                //
//                DebounceClearPressed(&DescriptBtnGPS);
//            }
//            // Si appui sur le bouton Start Stop
//            //=================================================
//            if(DebounceIsPressed(&DescriptBtnStartStop))
//            {             
//                // Reset compteur de timeout
//                btnStartStopTime = 0;
//                if(btnStartStopCnt < NB_PRESS_BTN_MAX)
//                {
//                    btnStartStopCnt++;
//                }
//                DebounceClearPressed(&DescriptBtnStartStop);
//            }
            //========================================================
            // Mesure du niveau de la batterie tous les 20 ms
            //========================================================
            V_Bat_Mes_Cnt = (V_Bat_Mes_Cnt + 1) % 200;
            if(V_Bat_Mes_Cnt == 0)
            {
                EN_ReadBATOn();
            }
            else if(V_Bat_Mes_Cnt == 100)
            {
                V_Bat = ReadAllAnalogValues();
                EN_ReadBATOff();
                if(SPI_GetState() == SPI_STATE_IDLE)
                {
                    LCD_EADOGS_GoTo(&RegVal, 1, 1);
                    LCD_Printf("Niveau Batterie     %1.2f",V_Bat);
                }
            }
            //========================================================
            // Affichage du niveau de batterie
            //========================================================
            // Pendant la charge
            if((V_Bat <= BAT_CHARGING) && (V_Bat > BAT_100))
            {
                BatLvl_ROn();
                BatLvl_GOn();
                BatLvl_BOn();
            }
            // Entre 100% et 90%
            else if((V_Bat <= BAT_100) && (V_Bat >= BAT_90))
            {
                BatLvl_ROff();
                BatLvl_GOn();
                BatLvl_BOff();
            }
            // Entre 89% et 50%
            else if(V_Bat >= BAT_50)
            {
                BatLvl_ROn();
                BatLvl_GOff();
                BatLvl_BOn();
            }
            // Entre 49% et 30%
            else if(V_Bat >= BAT_30)
            {
                BatLvl_ROn();
                BatLvl_GOn();
                BatLvl_BOff();
            }
            // Entre 29% et 10%
            else if(V_Bat >= BAT_10)
            {
                BatLvl_ROn();
                BatLvl_GOff();
                BatLvl_BOff();
            }
            // Entre 9% et 0%
            else if(V_Bat < BAT_10)
            {   
                if(blinkCnt == (BLINK_PER / 2))
                {
                    BatLvl_ROn();
                }
                else if(blinkCnt == BLINK_PER)
                {
                    BatLvl_ROff();
                }
                BatLvl_GOff();
                BatLvl_BOff();
            }
            //=========================================================
            // Detection de la présence de la carte SD
            //=========================================================
            // Si carte SD pas inseré
            // Allumer la LED rouge du voyant SDCard
            if(SDCard_DetectStateGet() == false)
            {
                SDCard_ROn();
                SDCardmounted = false;
            }
            // si non l'éteindre
            else
            {
                SDCard_ROff();
                SDCardmounted = true;
            }
            
            //========================================================
            //                       Machine d'état
            //========================================================
            switch(appData.tracker_State)
            {
                //=====================================================
                //                      Idle
                //=====================================================
                case INIT:
                    
                    if(Cnt == 1)
                    {
//                        if(SPI_GetState() == SPI_STATE_IDLE)
//                        {
//                            LCD_EADOGS_GoTo(&RegVal, 1,3);
//                            LCD_Printf("2308A TrackerGPS");
//                            LCD_Printf("Einar Farinas");
//                        }
                    }
//                    else if(Cnt == 10)
//                    {
//                        LCD_EADOGS_GoHome(&RegVal);
//                    }
                    
                    if(Cnt == 250)
                    {
//                        LCD_EADOGS_GoHome(&RegVal);
                        appData.tracker_State = IDLE;
                    }
                    Cnt++;
                    break;
                //=====================================================
                //                      Idle
                //=====================================================
                case IDLE:
//                    if(SPI_GetState() == SPI_STATE_IDLE)
//                    {
//                        LCD_EADOGS_GoTo(&RegVal, 1, 4);
//                        LCD_Printf("IDLE");
//                    }
                    // Si appui 1 fois sur le bouton GPS
                    if(DescriptBtnGPS.nbTouch == 1)
                    {
//                        DescriptBtnGPS.nbTouch = 0;
                        // Passer dans l'état d'attente de connexion aux satellites
                        appData.tracker_State = WAIT_FOR_CONNECTION;
                    }
                    if(GetUsbMessage(USB_RxBuffer, 9))
                    {
                        if(strstr((char*)USB_RxBuffer, (char*)"!Connect#") != NULL)
                        {
                            USB_Connected = true;
                            SendUsbMessage(USB_RxBuffer,20);
                            appData.tracker_State = USB_IDLE;
                        }
                    }
                    
                    // Affichage état idle
                    
//                    if(SPI_GetState() == SPI_STATE_IDLE)
//                    {
//                        LCD_Printf("2308A Tracker GPS");
//                        LCD_CursorHome(&RegVal);
//                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
//                        LCD_Printf("Einar Farinas");
//                        appData.tracker_State = WAIT_FOR_CONNECTION;
//                    }
                    
//                    Cnt = (Cnt + 1) % 10;
//                    if(true)
//                    {
                    if((GetSDCardWriteFlag() == false) && (SDCardmounted == true))
                    {

                        dataSize = GetGnssCmd(GnssData);
                        if(dataSize != 0)
                        {
//                                SendUsbMessage(GnssData, dataSize);
                            SetBuffer(GnssData, dataSize);
                            SetSDCardWriteFlag();
                        }
//                            dataSize = ScanGnssCmd(MINMEA_SENTENCE_GLL, GnssData);
//                            if(dataSize != 0)
//                            {
//                                SetBuffer(GnssData, dataSize);
//                                SetSDCardWriteFlag();
//                            }
                        ResetBuffer(GnssData);
                    }
//                    }
//                    if(GetSDCardWriteFlag() == false)
//                    {
//                        ResetBuffer(GnssData);
//                        dataSize = ScanGnssCmd(MINMEA_SENTENCE_GLL, GnssData);
//                        if(dataSize != 0)
//                        {
//                            SetBuffer(GnssData, dataSize);
//                            SetSDCardWriteFlag();
//                        }
//                    }
                    
                    
                    // Si le bouton Mode est appuyée pendant 3s
                    break;
                //=====================================================
                //          Attente de connection aux satellites
                //=====================================================
                case WAIT_FOR_CONNECTION:
                    // Attendre la réecpetion d'une cordonnée
                    
//                    LCD_Printf("GPS");
                    // clignotement LED GPS
                    
                    if(DescriptBtnGPS.nbTouch == 2)
                    {
//                        DescriptBtnGPS.nbTouch = 0;
                        // Passer dans l'état d'attente de connexion aux satellites
                        appData.tracker_State = READY;
                    }
                    break;
                //=====================================================
                // Connexion aux satellites réussie
                // système prêt à acquerir et sauvegarder des coordonnées
                //=====================================================
                case READY:
//                    LCD_Printf("READY");
                    // Si appui 1 fois sur le bouton GPS et le point de départ n'a pas été sauvegardé
                    if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == false))
                    {
                        // Retour à l'état idle
                        appData.tracker_State = IDLE;
//                        DescriptBtnGPS.nbTouch = 0;
//                        break;
                    }
                    // Si appui 1 fois sur le bouton GPS et et point de départ a été sauvegardé
                    else if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == true))
                    {
                        // Retour à l'état run pour l'acquisition des coordonnées
                        appData.tracker_State = RUN;
                        // Reset flag
                        startPointSet = false;
//                        DescriptBtnGPS.nbTouch = 0;
//                        break;
                    }
                    // Si appui sur le bouton Start
                    else if((DescriptBtnStartStop.nbTouch == 1) && (startPointSet == false))
                    {
                        // Set le flag du point de départ
                        startPointSet = true;
                        DescriptBtnStartStop.nbTouch = 0;
                        // Savegarde la coordonnée de départ 
                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    break;
                //=====================================================
                // Acquisition et savegarde de coordonnées selon l'intervalle de temps
                //===================================================== 
                case RUN:
//                    LCD_Printf("RUN");
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
                    if((DescriptBtnStartStop.nbTouch == 1)  && (endPointSet == false))
                    {
                        // Set flag du point d'arrivée
                        endPointSet = true;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    // Si flag de point d'arrivé set et appui 2 fois sur 
                    // bouton GPS
                    // Confirmer le point d'arrivé
                    else if((DescriptBtnStartStop.nbTouch == 2) && (endPointSet == true))
                    {
                        // Savegarde la coordonnée de départ 
                        appData.tracker_State = SAVE_COORDINATES;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    else if(DescriptBtnStartStop.nbTouch != 0)
                    {
                        // Reset flag du point d'arrivée
                        endPointSet = false;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    break;
                //=====================================================
                // Savegarde coordonnées dans la carte SD
                //=====================================================
                case SAVE_COORDINATES:
//                    LCD_Printf("SAVECOORD");
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
//                    LCD_Printf("USB_IDLE");
                    if(GetUsbMessage(USB_RxBuffer, 6))
                    {
                        if(strstr((char*)USB_RxBuffer, (char*)"!Read#") != NULL)
                        {
                            SetSDCardReadFlag();
                            appData.tracker_State = USB_READ_DATA;
                        }
                    }
                    else if(GetUsbMessage(USB_RxBuffer, 11))
                    {
                        if(strstr((char*)USB_RxBuffer, (char*)"!KeepAlive#") != NULL)
                        {
                            KeepAliveCnt = 0;
                        }
                    }
                    if(DescriptBtnStartStop.nbTouch == 1)
                    {             
                        SetSDCardReadFlag();
                        appData.tracker_State = USB_READ_DATA;
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                            
//                    if(KeepAliveCnt < USB_TIME_OUT)
//                    {
//                        KeepAliveCnt++;
//                    }
//                    else
//                    {
//                        KeepAliveCnt = 0;
//                        appData.tracker_State = IDLE;
//                    }
                    
                    break;
                // Lecture et récupération des données sur la carte SD
                case USB_READ_DATA:
//                    LCD_Printf("USB_READ");
                    // Attente que la lecture du fichier soit finie
                    if(GetSDCardReadFlag() == false)
                    {
                        appData.tracker_State = USB_SEND_DATA;
                    }
                    break;
                // Envoi des données récupérées par USB au PC
                case USB_SEND_DATA:
//                    LCD_Printf("USB_SEND");
//                    GetCmdToSend(USB_TxBuffer);
//                    SendUsbMessage(USB_TxBuffer, 510);
                    if(GetEndReadFlag() == false)
                    {
                        appData.tracker_State = USB_IDLE;
                    }
                    
                    break;
                // Mode configuration 
                // Pour régler l'intervalle de temps ou autres paramètres
                case CONFIG:
//                    LCD_Printf("CONFIG");
                     // Selection du digit
                    //======================
                    if(DescriptBtnMode.nbTouch == 1)
                    {
                        // Deplacement du curseur au digit suivante
//                        DescriptBtnMode.nbTouch = 0;
                    }
                    else if(DescriptBtnMode.nbTouch != 0)
                    {
//                        DescriptBtnMode.nbTouch = 0;
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
                    if(DescriptBtnStartStop.nbTouch == 1)
                    {
                        if(timeSpan <= timeSpanMaxVal)
                        {
                            timeSpan += timeSpanIncrDecrValue;
                        }
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    else if(DescriptBtnStartStop.nbTouch != 0)
                    {
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    // Décrémentation de la valeur du digit
                    // avec le boutons GPS
                    if(DescriptBtnGPS.nbTouch == 1)
                    {
                        if(timeSpan >= timeSpanMinVal)
                        {
                            timeSpan -= timeSpanIncrDecrValue;
                        }
//                        DescriptBtnGPS.nbTouch = 0;
                    }
                    else if(DescriptBtnGPS.nbTouch != 0)
                    {
//                        DescriptBtnGPS.nbTouch = 0;
                    }
                    break;
                //=====================================================
                // Pour sauvegarder la configuration choisie
                //=====================================================
                case SAVE_CONFIG:
                   
                    
                    break;
                default:
                    break;
                    
            }
            DescriptBtnGPS.nbTouch = 0;
            DescriptBtnStartStop.nbTouch = 0;  
            DescriptBtnMode.nbTouch = 0;
            
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


void ScanButtons(S_SwitchDescriptor *DescriptButton, bool ButtonState)
{
    // Bouton mode config
    DoDebounce(DescriptButton, ButtonState);
    //=================================================
    // Gestion du nombre d'appuis sur les boutons
    //=================================================
    DescriptButton->TimeoutCnt = (DescriptButton->TimeoutCnt + 1) % TIMEBTN;
    

    // Si appui sur le bouton Mode config
    //=================================================
    if(DebounceIsPressed(DescriptButton))
    {             
        // Reset compteur de timeout
        DescriptButton->TimeoutCnt = 0;

        if(DescriptButton->nbTouchCnt < NB_PRESS_BTN_MAX)
        {
            DescriptButton->nbTouchCnt++;
        }
        // 
        DebounceClearPressed(DescriptButton);
    }
    
    if(DescriptButton->TimeoutCnt == (TIMEBTN - 1))
    {
        DescriptButton->nbTouch = DescriptButton->nbTouchCnt;
        DescriptButton->nbTouchCnt = 0;
    }
}
/*******************************************************************************
 End of File
 */
