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
#include "Mc32Delays.h"

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

uint8_t R_Ton;
uint8_t G_Ton;
uint8_t B_Ton;

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
    static uint8_t GnssData[90] = {'$','G','N','R','M','C',',','1','2','4','8','3','8','.','0','0',',','A',',','4','6','3','1','.','5','0','7','0','6',',','S',',','0',
                                   '0','6','3','6','.','9','7','8','1','9',',','W',',','0','.','1','7','9',',',',','2','1','0','9','2','3',',',',',',','A','*','6','8'};
    
    //                 Compteurs
    //=====================================================
    // Compteur pour le temps d'envoie des données
    static uint8_t Cnt = 0;
    uint8_t dataSize = 0;
    
    //                  Batterie
    //=====================================================
    // Variable pour la tension de la batterie
    static float V_Bat = 0;
    
    //               Intervalle de sauvegarde
    //=====================================================
    // Consigne de l'intervalle en [s], valeur par défaut au min (30s)
    // valeur max 15 min = 900s
    static uint16_t timeSpan = 10;
    uint8_t timeSpanMin = 0;
    uint8_t timeSpanSec = timeSpan;
    // Compteur de temps pour l'intervalle en [s]
    static uint16_t timeSpanCnt = 1;
    // Variable pour la position du digit à modifier de l'intervalle dans le mode config
    // Ex pour interevalle de  12:34
    // 0 => 4
    // 1 => 3
    // 2 => 2
    // 3 => 1
    static uint8_t timeSpanDigit = 0;
    static uint8_t digitPos = 7;
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
    static s_EADOGS_REG_VAL RegVal;
    RegVal.displayControl = DISPLAY_CONTROL;
    RegVal.extendedFunctionSet = EXTENDED_FUNCTION_SET;
    RegVal.functionSet_RE0 = FUNCTIONSET_RE0;
    RegVal.functionSet_RE1 = FUNCTIONSET_RE1;
    RegVal.power_Display_Contrast = POWER_DISPLAY_CONTRAST;
    
    static bool newPrint = false;
    static bool updatePrint = false;
    
    static MENU_STATES Menu = START_MENU_1;
    static uint8_t backLightLvl = 10;
    
    //               Flags
    //=====================================================
    // Point de départ
    static bool startPointSet = false;
    // Point d'arrivé
    static bool endPointSet = false;
    bool SDCardmounted;
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
    
    //                  GNSS
    //=====================================================
    static minmea_messages Messages;
	static s_CoordinatesParse Latitude;
	static s_CoordinatesParse Longitude;
    uint32_t valTemp;
    static bool getCoordinate = false;
    
    // Compteur pour le clignotement des LEDs
    static uint8_t blinkCnt = 0;
    
    
    //                      Buzzer
    //=====================================================
    static bool enableBuzzer = false;
//    SPI_DoTasks();
    
    /* Check the application's current state. */
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // Démarrage des timers 
            DRV_TMR0_Start();  // Pour le programme principale
            DRV_TMR1_Start();  // Pour l'OC du backlight
            DRV_TMR2_Start();  // Pour l'OC du Buzzer
            // Démarrage des OCs
            DRV_OC0_PulseWidthSet(4999); // pour 100%
            DRV_OC0_Start();   // Pour le backlight de l'écran LCD
            // Initialisation du ADC
            InitADC10();
            // Démarrer une lecture
            ReadAllAnalogValues();
            ReadBattery(&V_Bat);
            // Initialisation des FIFOS pour les données reçus par le récepteur GPS
            // et les données à envoyer au PC
            InitFifoComm();                   
            // Initialisation des antirebonds sur les boutons 
            DebounceInit(&DescriptBtnMode);
            DebounceInit(&DescriptBtnGPS);
            DebounceInit(&DescriptBtnStartStop);
            // Reset buffer de datas du récepteur GNSS
            ResetBuffer(GnssData);
            
            // Faire l'init jusqu'à que le LCD soit initialisé
            if(SPI_GetState() == SPI_STATE_READY)
            {
                // Initialisation de l'écran LCD
                LCD_EADOGS_Init(&RegVal, BUS_8BITS, 4, BOTTOM_VIEW, false, true);
                // Curseur off
                LCD_BlinkCursor(&RegVal, true);
                // Passer dans l'état d'attente
                APP_UpdateState(APP_STATE_WAIT);
            }
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
            // Gestion du nombre d'appuis sur les boutons ave
            // l'antirebonds
            //=========================
            // Bouton mode config
            ScanButtons(&DescriptBtnMode, BtnModeStateGet());
            // Bouton GPS
            ScanButtons(&DescriptBtnGPS, BtnGPSStateGet());
            // Bouton Start / Stop
            ScanButtons(&DescriptBtnStartStop, BtnStartStopStateGet()); 
                
            // Compteur pour le clignotement des LEDs
            blinkCnt = (blinkCnt + 1) %  BLINK_PER;

            //========================================================
            // Mesure du niveau de la batterie tous les 3s
            // temps defini avec BAT_TIME (=300 pour 3s)
            //========================================================
            ReadBattery(&V_Bat);
            
            //========================================================
            // Affichage sur la LEDs du niveau de batterie
            //========================================================
            // Entre 100% et 90%
            if((V_Bat <= BAT_100) && (V_Bat >= BAT_90))
            {
                // Allume en vert
                R_Ton = 0;
                G_Ton = 2;
                B_Ton = 0;
            }
            // Entre 89% et 50%
            else if(V_Bat >= BAT_50)
            {
                // Allumer en orange
                R_Ton = 2;
                G_Ton = 1;
                B_Ton = 0;
            }
            // Entre 49% et 30%
            else if(V_Bat >= BAT_30)
            {
                // Allumer en jaune
                R_Ton = 2;
                G_Ton = 2;
                B_Ton = 0;
            }
            // Entre 29% et 10%
            else if(V_Bat >= BAT_10)
            {
                // Allumer en rouge
                R_Ton = 2;
                G_Ton = 0;
                B_Ton = 0;
            }
            // Entre 9% et 0%
            else if(V_Bat < BAT_10)
            {   
                // clignoter en rouge
                if(blinkCnt == (BLINK_PER / 2))
                {
                    R_Ton = 2;
                
                }
                else if(blinkCnt == (BLINK_PER-1))
                {
                    R_Ton = 0;
                }
                G_Ton = 0;
                B_Ton = 0;
            }
            // Si 0%
            else if(V_Bat <= BAT_0)
            {
                // Eteindre périphériques
                DRV_OC0_Stop();
                DRV_OC1_Stop();
                DRV_TMR1_Stop();
                DRV_TMR2_Stop();
                DRV_USART0_Deinitialize();
                DRV_USART1_Deinitialize();
                // Desactiver LCD
                LCD_EADOGS_Sleep(&RegVal);
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
                
                // Passer dans l'état OFF
                APP_UpdateState(APP_STATE_OFF);
                break;
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
                    // Affichage de démarrage pendant 2,5s
                    if(Cnt == 50)
                    {
                        if(SPI_GetState() == SPI_STATE_READY)
                        {
                            LCD_EADOGS_GoTo(&RegVal, 1,1);
                            LCD_Printf("2308 TrackerGPS");
                            LCD_EADOGS_GoTo(&RegVal, 1,2);
                            LCD_Printf("Einar Farinas");
                        }
//                        DRV_OC1_Start();
                    }
                    // Son de démarrage
//                    if((Cnt % 20) == 0)
//                    {
//                        DRV_TMR2_PeriodValueSet(buzzerNotes[buzzerCnt]);
//                        DRV_OC1_PulseWidthSet((buzzerNotes[buzzerCnt])/2);
//                        buzzerCnt = (buzzerCnt + 1) % 3;
//                    }
                    // Après 2,5 secondes
                    if(Cnt == 250)
                    {
                        // set flag d'affichage pour un nouveau affichage
                        newPrint = true;
                        // Passser dans l'état IDLE
                        appData.tracker_State = IDLE;
//                        DRV_OC1_Stop();
                    }
                    Cnt++;
                    break;
                //=====================================================
                //                      IDLE
                //=====================================================
                case IDLE:
                    // Si nouveau affichage et le SPI est IDLE
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage du nom de l'état
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
                        LCD_Printf("      IDLE     ");
                        // Reset flag d'affichage
                        newPrint = false;
                    }
                    // Si appui 1 fois sur le bouton GPS
                    if(DescriptBtnGPS.nbTouch == 1)
                    {
                        // Set flag d'affichage
                        newPrint = true;
                        // Passer dans l'état d'attente de connexion aux satellites
                        appData.tracker_State = WAIT_FOR_CONNECTION;
                    }
                    // S'il y a des données dans le FIFO USB 
                    if(GetUsbMessage(USB_RxBuffer, 9))
                    {
                        // Si la commande !Connect# a été reçu
                        if(strstr((char*)USB_RxBuffer, (char*)"!Connect#") != NULL)
                        {
                            USB_Connected = true;
                            // Envoyer une quittance
                            SendUsbMessage(USB_RxBuffer,20);
                            // Passer dans l'état USB IDLE
                            appData.tracker_State = USB_IDLE;
                        }
                    }
                    // Si appui sur le bouton MODE pendant 3s
                    if(DescriptBtnMode.HoldCnt == 300)
                    {
                        // Set flag d'affichage
                        newPrint = true;
                        // Passer dans l'état de configuration
                        appData.tracker_State = CONFIG;
                    }
                    
                    
                    // pour test le Buzzer en appuyant sur le bouton MODE
                    //---------------------------------------------------------------------
                    if(DescriptBtnMode.nbTouch == 1)
                    {
                        enableBuzzer = !enableBuzzer;
                    }
                    if(enableBuzzer == true)
                    {
                        enableBuzzer = !(BuzzerPlay(100, 0));
//                        {
//                            enableBuzzer = false;
//                        }
//                        Cnt = (Cnt + 1) % 10;
//                        if(Cnt == 0)
//                        {
//                            DRV_TMR2_PeriodValueSet(buzzerNotes[buzzerCnt]);
//                            DRV_OC1_PulseWidthSet((buzzerNotes[buzzerCnt])/10);
//                            buzzerCnt = (buzzerCnt + 1) % 7;
//                        }
                    }
                    //---------------------------------------------------------------------
                    
//                    Cnt = (Cnt + 1) % 10;
//                    if(true)
//                    {
                    
                    
                    
//                    Cnt = (Cnt + 1) % 100;
//                    if(Cnt == 0)
//                    {
//                        timeSpanCnt = (timeSpanCnt + 1) % timeSpan;
//                        if(timeSpanCnt == 0)
//                        {
//                            waitForCmd = true;
//                        }
//                    }
//                    
////                    dataSize = GetGnssCmd(GnssData);
//                    
//                    if((((GetSDCardWriteFlag() == false) && (SDCardmounted == true)) ))// || (waitForCmd == true)))
//                    {
//                        dataSize = GetGnssCmd(GnssData);
//                        if(dataSize != 0)
//                        {
////                                SendUsbMessage(GnssData, dataSize);
//                            SetBuffer(GnssData, dataSize);
//                            SetSDCardWriteFlag();
//                        }
////                            dataSize = ScanGnssCmd(MINMEA_SENTENCE_GLL, GnssData);
////                        if(dataSize != 0)
////                        {
////                            if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_RMC)
////                            {
////                                SetBuffer(GnssData, dataSize);
////                                SetSDCardWriteFlag();
////                                waitForCmd = false;
////                            }
////                        } 
//                    }
//                    ResetBuffer(GnssData);
                    
                    
                    
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
                    
                    break;
                //=====================================================
                //          Attente de connection aux satellites
                //=====================================================
                case WAIT_FOR_CONNECTION:
                    // Si nouveau affichage et le SPI est en IDLE
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage de l'état d'attente de connexion 
                        // avec le nombre de satellites en vue
                        // et la valeur de CN0
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal,1,1);
                        LCD_Printf("Connecting...");
                        LCD_EADOGS_GoTo(&RegVal,1,2);
                        LCD_Printf("Sat=%2d", Messages.gsv.total_sats);
                        LCD_EADOGS_GoTo(&RegVal,1,3);
                        LCD_Printf("CNO=%2d %2d %2d %2d", Messages.gsv.sats[0].snr,
                                Messages.gsv.sats[1].snr, Messages.gsv.sats[2].snr,
                                Messages.gsv.sats[3].snr);
                        // Reset flag d'affichage
                        newPrint = false;
                    }
                    if((SPI_GetState() == SPI_STATE_READY) && (updatePrint == true))
                    {
                        // Affichage de l'état d'attente de connexion 
                        // avec le nombre de satellites en vue
                        // et la valeur de CN0
                        LCD_EADOGS_GoTo(&RegVal,5,2);
                        LCD_Printf("%2d", Messages.gsv.total_sats);
                        LCD_EADOGS_GoTo(&RegVal,5,3);
                        LCD_Printf("%2d %2d %2d %2d", Messages.gsv.sats[0].snr,
                                Messages.gsv.sats[1].snr, Messages.gsv.sats[2].snr,
                                Messages.gsv.sats[3].snr);
                        // Reset flag d'affichage
                        updatePrint = false;
                    }
                    // Clignotement LED verte GPS
                    // indiquer l'attente de connexion aux satellites
                    if(blinkCnt == (BLINK_PER / 2))
                    {
                        GPS_GOn();
                    }
                    else if(blinkCnt == (BLINK_PER -1))
                    {
                        GPS_GOff();
                    }
                    // Récupération d'une commande dans le FIFO Gnss
                    dataSize = GetGnssCmd(GnssData);
                    // Si la commande a été récupérée
                    if(dataSize != 0)
                    {
                        // Si c'est une commande xxGSV
                        if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_GSV)
                        {
                            // récupération des valeurs dans la commande
                            minmea_parse_gsv(&Messages.gsv, (char*)GnssData);
                        }
                        // Si c'est une commande xxRMC
                        else if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_RMC)
                        {
                            // Récupération des valeurs dans la commande
                            minmea_parse_rmc(&Messages.rmc, (char*)GnssData);
                            // Si des coordonnées on été reçues
                            if((Messages.rmc.latitude.value != 0) && (Messages.rmc.longitude.value != 0))
                            {
                                // Set flag affichage
                                newPrint = true;
                                // Passer dans l'état READY
                                appData.tracker_State = READY;
                                enableBuzzer = true;
                                //------------------------------------
                                // Pour Test
//                                appData.tracker_State = TRACKING;
                                //------------------------------------
                                // Allumer la LED verte GPS
                                GPS_GOn();
                            }
                        }
                        // Set flag d'affichage
                        // après réception de 20 commandes
                        Cnt = (Cnt + 1) % 20;
                        if(Cnt == 0)
                        {
                            updatePrint = true;
                        }
                    }
                    // ----------------------------------
                    // Pour Test
                    if(DescriptBtnGPS.nbTouch == 1)
                    {
                        // Set flag affichage
                        newPrint = true;
                        appData.tracker_State = TRACKING;
                    }
                    // ----------------------------------
                    // Reset Buffer
                    ResetBuffer(GnssData);
                    
                    // Si appui sur le bouton MODE pendant 3s
                    if(DescriptBtnMode.HoldCnt == 300)
                    {
                        // Set flag d'affichage
                        newPrint = true;
                        // Passer dans l'état de configuration
                        appData.tracker_State = CONFIG;
                    }
                    break;
                //=====================================================
                // Connexion aux satellites réussie
                // système prêt à acquerir et sauvegarder des coordonnées
                //=====================================================
                case READY:
                    // Si nouveau affichage et SPI en IDLE
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage de l'état READY
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 3, 2);
                        LCD_Printf("READY");
                        newPrint = false;
                    }
                    // Si appui 1 fois sur le bouton GPS et le point de départ n'a pas été sauvegardé
                    if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == false))
                    {
                        // Retour à l'état IDLE
                        appData.tracker_State = IDLE;
//                        DescriptBtnGPS.nbTouch = 0;
                        // Set flag d'affichage
                        newPrint = true;
                    }
                    // Si appui 1 fois sur le bouton GPS et et point de départ a été sauvegardé
                    else if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == true))
                    {
                        // Retour à l'état TRACKING pour l'acquisition des coordonnées
                        appData.tracker_State = TRACKING;
                        // Reset flag du point de départ
                        startPointSet = false;
//                        DescriptBtnGPS.nbTouch = 0;
                        // Set flag d'affichage
                        newPrint = true;
                    }
                    // Si appui sur le bouton Start et le point de départ n'a pas été sauvegardée
                    else if((DescriptBtnStartStop.nbTouch == 1) && (startPointSet == false))
                    {
                        // Set le flag du point de départ
                        startPointSet = true;
//                        DescriptBtnStartStop.nbTouch = 0;
                        // Set flag d'affichage
                        newPrint = true;
                        // Savegarde la coordonnée de départ 
//                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    
                    
                    // Si appui sur le bouton MODE pendant 3s
                    if(DescriptBtnMode.HoldCnt == 300)
                    {
                        // Set flag d'affichage
                        newPrint = true;
                        // Passer dans l'état de configuration
                        appData.tracker_State = CONFIG;
                    }
                    break;
                //=====================================================
                // Acquisition et savegarde de coordonnées selon l'intervalle de temps
                //===================================================== 
                case TRACKING:
                    // Si nouveau affichage et SPI en IDLE
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Afifchage de l'heure, la date et les coordonnées
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 1);
                        LCD_Printf("Tracking");
                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
                        LCD_Printf("%02d:%02d %02d/%02d/%02d", Messages.rmc.time.hours, Messages.rmc.time.minutes, Messages.rmc.date.day, Messages.rmc.date.month, Messages.rmc.date.year);
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("?%02d%c%02d'%2.2f\"", Latitude.degrees, 0xDF, Latitude.min, Latitude.sec);
                        LCD_EADOGS_GoTo(&RegVal, 1, 4);
                        LCD_Printf("?%02d%c%02d'%2.2f\"", Longitude.degrees, 0xDF, Longitude.min, Longitude.sec);
                        newPrint = false;
                    }
                    if((SPI_GetState() == SPI_STATE_READY) && (updatePrint == true))
                    {
                        // Afifchage de l'heure, la date et les coordonnées
                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
                        LCD_Printf("%02d:%02d %02d/%02d/%2d", Messages.rmc.time.hours, Messages.rmc.time.minutes, Messages.rmc.date.day, Messages.rmc.date.month, Messages.rmc.date.year);
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("%c%02d%c%02d'%2.2f\"", Latitude.direction, Latitude.degrees, 0xDF, Latitude.min, Latitude.sec);
                        LCD_EADOGS_GoTo(&RegVal, 1, 4);
                        LCD_Printf("%c%02d%c%02d'%2.2f\"", Longitude.direction, Longitude.degrees, 0xDF, Longitude.min, Longitude.sec);
                        updatePrint = false;
                    }
                    // Compteur pour 1s
                    Cnt = (Cnt + 1) % 100;
                    // Si une seconde est passée
                    if(Cnt == 0)
                    {
                        // Incrémenter compteur de l'intervalle de temps
                        timeSpanCnt = (timeSpanCnt + 1) % timeSpan;
                    }
                    // Si intervalle de temps écoulé
                    if((timeSpanCnt == (timeSpan - 1)) && (getCoordinate == false) && (Cnt == 0))
                    {
                        // Set flag pour la récupération des coordonnées
                        getCoordinate = true;
                    }
                    // Récupération des commandes dans le FIFO pour le vider
                    dataSize = GetGnssCmd(GnssData);
                    // récupération des coordonnées
                    if(getCoordinate == true)
                    {
                        // Si une commande a été récupérée
                        if(dataSize != 0)
                        {
                            // Si la commande est xxRMC
                            if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_RMC)
                            {
                                // Récupération des valeurs dans la commande
                                minmea_parse_rmc(&Messages.rmc, (char*)GnssData);
                            }
                            // Si la commande est valide
                            if(Messages.rmc.valid == true)
                            {
                                // Set flag d'affichage
                                updatePrint = true;
								// Conversion de la valeur des coordonnées
                                //-----------------------------------------------------------------
                                // Direction => Nord ou Sud
                                if(Messages.rmc.latitude.value >= 0)
                                {
                                   Latitude.direction = 'N';
                                   valTemp = Messages.rmc.latitude.value;
                                }
                                else
                                {
                                   Latitude.direction = 'S';
                                   valTemp = Messages.rmc.latitude.value * -1;
                                }
                                // Latitude
								Latitude.degrees = valTemp / 10000000;
                                valTemp = (valTemp - (Latitude.degrees * 10000000));
								Latitude.min = valTemp / 100000;
                                valTemp = valTemp - (Latitude.min *100000);
								Latitude.sec = (float)(valTemp * 60) / 100000.0;
								
                                // Direction Est ou Ouest
                                if(Messages.rmc.longitude.value >= 0)
                                {
                                   Longitude.direction = 'E';
                                   valTemp = Messages.rmc.longitude.value;
                                }
                                else
                                {
                                   Longitude.direction = 'W';
                                   valTemp = Messages.rmc.longitude.value * -1;
                                }
                                // Longitude
								Longitude.degrees = valTemp / 10000000;
                                valTemp = (valTemp - (Longitude.degrees * 10000000));
								Longitude.min = valTemp / 100000;
                                valTemp = valTemp - (Longitude.min *100000);
								Longitude.sec = (float)(valTemp * 60) / 100000.0;
                                // Activer Buzzer
                                enableBuzzer = true;
                                // Reset flag
                                getCoordinate = false;
                                // Passer dans l'état de savuegarde sur la carte SD
//                                appData.tracker_State = SAVE_COORDINATES;
                            }
                        }
                    }
                    // Reset Buffer
                    ResetBuffer(GnssData);
                    // Si appui une première fois sur le bouton start/stop
                    // et le falg du point d'arrivée est false
                    if((DescriptBtnStartStop.nbTouch == 1)  && (endPointSet == false))
                    {
                        // Set flag du point d'arrivée
                        endPointSet = true;
                    }
                    // Si flag de point d'arrivé set et appui 2 fois sur 
                    // bouton GPS  et le flag du point d'arrivée est true
                    
                    else if((DescriptBtnStartStop.nbTouch == 2) && (endPointSet == true))
                    {
                        // Confirmer le point d'arrivé et enregistrer le point
                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    // dans les autres cas si reset le flag du point d'arrivée
                    else if(DescriptBtnStartStop.nbTouch != 0)
                    {
                        // Reset flag du point d'arrivée
                        endPointSet = false;
                    }
                    
                    
                    // Si appui sur le bouton MODE pendant 3s
                    if(DescriptBtnMode.HoldCnt == 300)
                    {
                        // Set flag d'affichage
                        newPrint = true;
                        // Passer dans l'état de configuration
                        appData.tracker_State = CONFIG;
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
                        // Retourner dans l'état TRACKING
                        appData.tracker_State = TRACKING;
                    }
                    break;
                //=====================================================
                // Lors de la connection par USB au PC
                //=====================================================
                // Attente de demande de lecture des données sur la carte SD
                case USB_IDLE:
                    // Si nouveau affichage et SPI en IDLE
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage du mode USB
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 1);
                        LCD_Printf("USB MODE");
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("IDLE");
                        newPrint = false;
                    }
                    // Si 6 bytes (taille de la commande !Read#) dans le FIFO USB
                    if(GetUsbMessage(USB_RxBuffer, 6))
                    {
                        // Si réception de la commande !Read#
                        if(strstr((char*)USB_RxBuffer, (char*)"!Read#") != NULL)
                        {
                            // Set flag de lecture de la carte SD
                            SetSDCardReadFlag();
                            // Passer dans l'état de lecture
                            appData.tracker_State = USB_READ_DATA;
                        }
                    }
                    // Si 1 bytes (taille de la commande !KeepAlive#) dans le FIFO USB
                    else if(GetUsbMessage(USB_RxBuffer, 11))
                    {
                        // Si réception de la commande !KeepAlive#
                        if(strstr((char*)USB_RxBuffer, (char*)"!KeepAlive#") != NULL)
                        {
                            // Reset compteur keepAlive
                            KeepAliveCnt = 0;
                        }
                    }
                    // Pour test avec U-Center
                    // ------------------------------------------------------------------
                    // Se connecter avec PuTTY pour entrer dans l'état IDLE USB
                    // avec la commande !Connect#. Puis fermer PuTTY et ouvrir
                    // U-Center. Ouvrir le port et appuyer sur le bouton start/Stop
                    // pour envoyer les données de la carte SD sur U-Center
                    if(DescriptBtnStartStop.nbTouch == 1)
                    {             
                        SetSDCardReadFlag();
                        newPrint = true;
                        appData.tracker_State = USB_READ_DATA;
                    }
                    // ------------------------------------------------------------------
                    
                    // Incréemeter compteur keepAlive si plus petit que le timeout
                    if(KeepAliveCnt < USB_TIME_OUT)
                    {
                        KeepAliveCnt++;
                    }
                    // Si timeout du compteur KeepAlive
                    else
                    {
                        KeepAliveCnt = 0;
                        newPrint = true;
                        appData.tracker_State = IDLE;
                    }
                    break;
                // Lecture et récupération des données sur la carte SD
                case USB_READ_DATA:
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage du mode USB
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("Reading Data");
                        newPrint = false;
                    }
                    // Attente que la lecture du fichier soit finie
                    if(GetSDCardReadFlag() == false)
                    {
                        appData.tracker_State = USB_SEND_DATA;
                    }
                    break;
                // Envoi des données récupérées par USB au PC
                case USB_SEND_DATA:
                    // Pour test avec U-Center
                    //------------------------------------
//                    LCD_Printf("USB_SEND");
//                    GetCmdToSend(USB_TxBuffer);
//                    SendUsbMessage(USB_TxBuffer, 510);
                    //------------------------------------
                    if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                    {
                        // Affichage du mode USB
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("Sending Data ");
                        newPrint = true;
                    }
                    if(GetEndReadFlag() == false)
                    {
                        appData.tracker_State = USB_IDLE;
                        newPrint = true;
                    }
                    
                    break;
                // Mode configuration 
                // Pour régler l'intervalle de temps ou autres paramètres
                case CONFIG:
                    // Compteur de 0 à 7 ms
                    Cnt = (Cnt + 1) % INCREMENT_HOLD_SPAN;
                    // Menu de configuration
                    switch(Menu)
                    {
                        // Sélection de l'option de l'intervalle
                        case START_MENU_1:
                            // Affichage du menu
                            if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,2,1);
                                LCD_Printf("Config Mode");
                                LCD_EADOGS_GoTo(&RegVal,3,2);
                                LCD_Printf(">Intervalle<");
                                LCD_EADOGS_GoTo(&RegVal,4,3);
                                LCD_Printf("Backlight");
                                newPrint = false;
                            }
                            // Si 1 appui sur le bouton MODE
                            if(DescriptBtnMode.nbTouch == 1)
                            {
                                // Passer dans le menu de configuration 
                                // pour changer l'intervalle de temps
                                Menu = TIME_SPAN;
                                newPrint = true;
                            }
                            // Si 1 appui sur le bouton Start/Stop
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                // Passer dans l'option suivante
                                Menu = START_MENU_2;
                                newPrint = true;
                            }
                            // Si 1 appui sur le bouton GPS
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                // Passer dans l'option suivante
                                Menu = START_MENU_2;
                                newPrint = true;
                            }
                            // Si appuis sur le bouton mode pendant 3s
                            if(DescriptBtnMode.HoldCnt == 300)
                            {
                                newPrint = true;
                                // Passer dans l'état IDLE
                                appData.tracker_State = IDLE;
                            }
                            break;
                        // Sélection de l'option du rétroéclairage
                        case START_MENU_2:
                            // Affichage du menu
                            if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,2,1);
                                LCD_Printf("Config Mode");
                                LCD_EADOGS_GoTo(&RegVal,3,2);
                                LCD_Printf(">Backlight<");
                                LCD_EADOGS_GoTo(&RegVal,4,3);
                                LCD_Printf("Intervalle");
                                newPrint = false;
                            }
                            // Si 1 appui sur le bouton MODE
                            if(DescriptBtnMode.nbTouch == 1)
                            {
                                // Passer dans le menu de configuration 
                                // pour changer les options du backlight
                                Menu = BACKLIGHT;
                                newPrint = true;
                            }
                            // Si 1 appui sur le bouton Start/Stop
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                // Passer dans l'option suivante
                                Menu = START_MENU_1;
                                newPrint = true;
                            }
                            // Si 1 appui sur le bouton GPS
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                // Passer dans l'option suivante
                                Menu = START_MENU_1;
                                newPrint = true;
                            }
                            // Si appuis sur le bouton mode pendant 3s
                            if(DescriptBtnMode.HoldCnt == 300)
                            {
                                newPrint = true;
                                // Passer dans l'état IDLE
                                appData.tracker_State = IDLE;
                            }
                            break;
                        case 2:
                            // Affichage du menu pour la configuration de
                            // l'intervalle de temps
                            if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,1,2);
                                LCD_Printf("Mode config");
                                LCD_EADOGS_GoTo(&RegVal,1,3);
                                LCD_Printf("Intervalle : ");
                                LCD_EADOGS_GoTo(&RegVal,1,4);
                                LCD_Printf("%02d : %02d", timeSpanMin, timeSpanSec);
                                LCD_EADOGS_GoTo(&RegVal, digitPos,4);
                                newPrint = false;
                            }
                            // Mise a jour de la valeur affichée
                            if((SPI_GetState() == SPI_STATE_READY) && (updatePrint == true))
                            {
                                if(timeSpan >= 60)
                                {
                                    timeSpanMin = timeSpan/60;
                                    timeSpanSec = timeSpan - timeSpanMin * 60;
                                }
                                LCD_EADOGS_GoTo(&RegVal,1,4);
                                LCD_Printf("%02d : %02d ", timeSpanMin, timeSpanSec);
                                LCD_EADOGS_GoTo(&RegVal, digitPos,4);
                                updatePrint = false;
                            }
                            // Selection du digit
                            //===============================================
                            // Si 1 appui sur le bouton MODE
                            if(DescriptBtnMode.nbTouch == 1)
                            {
                                // Deplacement du curseur au digit suivant
                                timeSpanDigit = (timeSpanDigit + 1) % 4;
                                updatePrint = true;
                            }
                            // Sélection de la valeur à incrémenter ou décrémenter
                            // en fonction du digit sélectionné plus la position du
                            // digit sur écran
                            switch(timeSpanDigit)
                            {
                                // Secondes unitées
                                case 0:
                                    timeSpanIncrDecrValue = 1;
                                    digitPos = 7;
                                    break;
                                // Secondes dixièmes
                                case 1:
                                    timeSpanIncrDecrValue = 10;
                                    digitPos = 6;
                                    break;
                                // Minutes unitées
                                case 2:
                                    timeSpanIncrDecrValue = 60;
                                    digitPos = 2;
                                    break;
                                // Minutes dixièmes
                                case 3:
                                    timeSpanIncrDecrValue = 600;
                                    digitPos = 1;
                                    break;
                                default:
                                    break;
                            }
                            // Set les valeurs min et max pour pouvoir
                            // incrémeter ou décrémenter la valeur en fonction
                            // du digit sélectionnée
                            timeSpanMinVal = (TIME_SPAN_MIN + timeSpanIncrDecrValue);
                            timeSpanMaxVal = (TIME_SPAN_MAX - timeSpanIncrDecrValue);

                            // Incrémentation de la valeur du digit
                            // avec le bouton StartStop
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                if(timeSpan <= timeSpanMaxVal)
                                {
                                    timeSpan += timeSpanIncrDecrValue;
                                    updatePrint = true;
                                }
//                                DescriptBtnStartStop.nbTouch = 0;
                            }
                            // Si bouton start/stop mantenu appuyé pendant plus de 100ms 
                            else if(DescriptBtnStartStop.HoldCnt >= HOLD_MIN_TIME)
                            {
                                // Incrémenter la valeur tous les 7ms
                                if((timeSpan <= timeSpanMaxVal) && (Cnt == 0))
                                {
                                    timeSpan += timeSpanIncrDecrValue;
                                    updatePrint = true;
                                }
                            }
                            // Décrémentation de la valeur du digit
                            // avec le boutons GPS
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                if(timeSpan >= timeSpanMinVal)
                                {
                                    timeSpan -= timeSpanIncrDecrValue;
                                    updatePrint = true;
                                }
                            }
                            // Si bouton GPS mantenu appuyé pendant plus de 100ms
                            else if(DescriptBtnGPS.HoldCnt >= HOLD_MIN_TIME)
                            {
                                // Décrémenter la valeur tous les 7ms
                                if((timeSpan >= timeSpanMinVal) && (Cnt == 0))
                                {
                                    timeSpan -= timeSpanIncrDecrValue;
                                    updatePrint = true;
                                }
                            }
                            // Si 2 appuis sur le bouton MODE
                            if(DescriptBtnMode.nbTouchCnt == 2)
                            {
                                newPrint = true;
                                // Retourner au menu de sélection
                                Menu = START_MENU_1;
//                                DescriptBtnMode.nbTouchCnt = 0;
                            }
                            break;
                        case 3:
                            // Affichage du menu de configuration du rétroéclairage
                            if((SPI_GetState() == SPI_STATE_READY) && (newPrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,1,2);
                                LCD_Printf("Mode config");
                                DRV_OC0_PulseWidthSet(BL_100 * backLightLvl / 100);
                                LCD_EADOGS_GoTo(&RegVal,1,3);
                                LCD_Printf("Backlight : %2d %%", backLightLvl);
                                newPrint = false;
                            }
                            // Mise a jour du niveau de rétroéclaireage sur l'écran
                            if((SPI_GetState() == SPI_STATE_READY) && (updatePrint == true))
                            {
                                DRV_OC0_PulseWidthSet(BL_100 * backLightLvl / 100);
                                LCD_EADOGS_GoTo(&RegVal,13,3);
                                LCD_Printf("%2d %%", backLightLvl);
                                updatePrint = false;
                            }
                            // Si 1 appui sur le bouton Start/Stop 
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                // Incrémenter le niveau du rétroéclairage
                                // si < la valeur max (100)
                                if(backLightLvl < BACKLIGHT_LVL_MAX)
                                {
                                    backLightLvl++;
                                    updatePrint = true;
                                }
                            }
                            // Si bouton Start/Stop mantenu appuyé pendant plus de 100 ms
                            else if(DescriptBtnStartStop.HoldCnt >= 100)
                            {
                                // Incrémenter le niveau du rétroéclairage tous les 7ms
                                // si < la valeur max (100)
                                if((backLightLvl < 100) && (Cnt == 0))
                                {
                                    backLightLvl++;
                                    updatePrint = true;
                                }
                            }
                            // Si 1 appui sur le bouton GPS 
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                // Décrémenter le niveau du rétroéclairage
                                // si > la valeur min (0)
                                if(backLightLvl > BACKLIGHT_LVL_MIN)
                                {
                                    backLightLvl--;
                                    updatePrint = true;
                                }
                            }
                            // Si bouton GPS mantenu appuyé pendant plus de 100 ms
                            else if(DescriptBtnGPS.HoldCnt >= 100)
                            {
                                // Incrémenter le niveau du rétroéclairage tous les 7ms
                                // si > la valeur mIN (0)
                                if((backLightLvl > 0) && (Cnt == 0))
                                {
                                    backLightLvl--;
                                    updatePrint = true;
                                }
                            }
                            // Si 2 appuis sur le bouton MODE
                            if(DescriptBtnMode.nbTouchCnt == 2)
                            {
                                newPrint = true;
                                // Retourner au menu de sélection
                                Menu = START_MENU_2;
                            }
                            break;
                            
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
            // Reset nombre d'appuis sur les boutons
            DescriptBtnGPS.nbTouch = 0;
            DescriptBtnStartStop.nbTouch = 0;  
            DescriptBtnMode.nbTouch = 0;
            
            if(enableBuzzer == true)
            {
                enableBuzzer = !BuzzerPlay(100, 0);
            }
            // Passer dans l'état WAIT
            APP_UpdateState(APP_STATE_WAIT);
            
            
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_STATE_OFF :
            // Rien faire
            break;
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

// Callback du Timer1 pour la commande de la LED de la batterie
void Timer1_CallBack(void)
{
    // Compteur pour pour le changment de LEDS
    static uint8_t CntLed = 0;
    
    // Comtpteur de 0 à 100 pour chager de LED chaque 1 seconde (T timer = 10 ms) 
    
    if((CntLed == 0) && (R_Ton != 0))
    {
        BatLvl_ROn();
    }
    else if(CntLed == R_Ton)
    {
        BatLvl_ROff();
    }
    if((CntLed == 0) && (G_Ton != 0))
    {
        BatLvl_GOn();
    }
    else if(CntLed == G_Ton)
    {
        BatLvl_GOff();
    }
    if((CntLed == 0) && (B_Ton != 0))
    {
        BatLvl_BOn();
    }
    else if(CntLed == B_Ton)
    {
        BatLvl_BOff();
    }
    
    CntLed = (CntLed + 1) % LED_PER;
    
    // Pour test initial des LEDs lors de la mise en service
    
//    if(Cnt == 0)
//    {
//        SelectLed = (SelectLed + 1) % NB_ACTIONS;
//        switch(SelectLed)
//        {
//            case 0:
//                GPS_ROff();
//                BatLvl_BOn();
//                break;
//            case 1:
//                GPS_ROn();
//                GPS_GOff();
//                break;
//            case 2:
//                GPS_GOn();
//                GPS_BOff();
//                break;
//            case 3:
//                GPS_BOn();
//                SDCard_ROff();
//                break;
//            case 4:
//                SDCard_ROn();
//                SDCard_GOff();
//                break;
//            case 5:
//                SDCard_GOn();
//                SDCard_BOff();
//                break;
//            case 6:
//                SDCard_BOn();
//                BatLvl_ROff();
//                break;
//            case 7:
//                BatLvl_ROn();
//                BatLvl_GOff();
//                break;
//            case 8:
//                BatLvl_GOn();
//                BatLvl_BOff();
//                break;
//            default:
//                break;
//        }
//    }
    
    
}
// Fonction pour reset le buffer à 0
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

// Fonction pour compter le nombre d'appuis ainsi que le temps de maintient sur 
// sur bouton
void ScanButtons(S_SwitchDescriptor *DescriptButton, bool ButtonState)
{
    // Bouton mode config
    DoDebounce(DescriptButton, ButtonState);
    //=================================================
    // Gestion du nombre d'appuis sur les boutons
    //=================================================
    DescriptButton->TimeoutCnt = (DescriptButton->TimeoutCnt + 1) % TIMEBTN;
    

    // Si appui sur le bouton
    if(DebounceGetInput(DescriptButton) == 1)
    {
        // Incrémenter compteur de maintient
        DescriptButton->HoldCnt++;
    }
    // Si le bouton a été appuyé pendant moins de 800ms puis rélaché 
    if((DebounceIsReleased(DescriptButton)) && (DebounceIsPressed(DescriptButton)) && (DescriptButton->HoldCnt < 80))
    {             
        // Reset compteur de timeout
        DescriptButton->TimeoutCnt = 0;
        // Incréementer le compteur de nombre d'appuis
        if(DescriptButton->nbTouchCnt < NB_PRESS_BTN_MAX)
        {
            DescriptButton->nbTouchCnt++;
        }
        // Reset flags d'appui et relache
        DebounceClearPressed(DescriptButton);
        DebounceClearReleased(DescriptButton);
    }
    // Si le bouton a été appuyé pendant plus de 800ms puis rélaché
    else if((DebounceIsReleased(DescriptButton)) && (DebounceIsPressed(DescriptButton)) && (DescriptButton->HoldCnt >= 80))
    {
        // Reset compteur de maintient
        DescriptButton->HoldCnt = 0;
        // Reset flags d'appui et relache
        DebounceClearPressed(DescriptButton);
        DebounceClearReleased(DescriptButton);
    }

    // Si timeout
    if(DescriptButton->TimeoutCnt == (TIMEBTN - 1))
    {
        // Enregistrer le nombre d'appuis
        DescriptButton->nbTouch = DescriptButton->nbTouchCnt;
        // Reset le compteur de nombre d'appuis
        DescriptButton->nbTouchCnt = 0;
    }
}

// Fonction pour la tension sur la batterie tous les X secondes,
// en fonction de BAT_TIME(valeur en ms) 
void ReadBattery(float *VBat)
{
    // Compteur pour la mesure de tension de la batterie
    static uint16_t V_Bat_Mes_Cnt = 0;
    
    // Si compteur à 0, activer l'entrée de lecture pour la batterie
    if(V_Bat_Mes_Cnt == 0)
    {
        EN_ReadBATOn();
    }
    // Si compteur = 1
    if(V_Bat_Mes_Cnt == 1)
    {
        // Lire la tension sur la batterie
        *VBat = ReadAllAnalogValues();
        // Desactiver l'entréé de lecture de la batterie
        EN_ReadBATOff();
    }
    // Comtpeur de 0 à BAT_TIME 
    // Pour lire la batterie tous les Xs en fonction de BAT_TIME
    V_Bat_Mes_Cnt = (V_Bat_Mes_Cnt + 1) % BAT_TIME;
}
// Fonction pour activer le buzzer avec un note pour une durée en ms
bool BuzzerPlay(uint16_t duration, uint8_t note)
{
    static int16_t durationCnt = 0;
    uint16_t buzzerNotes[7] = {9555, 8512, 7583, 7158, 6377, 5681, 5061};
    
    if(durationCnt == 0)
    {
        DRV_TMR2_PeriodValueSet(buzzerNotes[note]);
        DRV_OC1_PulseWidthSet((buzzerNotes[note])/2);
        DRV_OC1_Start();
    }
    if(durationCnt < duration)
    {
        durationCnt++;
        return false;
    }
    else
    {
        DRV_OC1_Stop();
        durationCnt = 0;
        return true;
    }         
}

/*******************************************************************************
 End of File
 */
