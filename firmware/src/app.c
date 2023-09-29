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
static bool EnablePrint = false;
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
    // Buffer pour les donn�es r��ues par le r�cepteur GNSS
    static uint8_t GnssData[90];
    
    //                 Compteurs
    //=====================================================
    // Compteur pour le temps d'envoie des donn�es
    static uint8_t Cnt = 0;
    uint8_t dataSize = 0;
    
    //                  Batterie
    //=====================================================
    // Variable pour la tension de la batterie
    static float V_Bat = 0;
    
    //               Intervalle de sauvegarde
    //=====================================================
    // Consigne de l'intervalle en [s], valeur par d�faut au min (30s)
    // valeur max 15 min = 900s
    static uint16_t timeSpan = 30;
    // Compteur de temps pour l'intervalle en [s]
    static uint16_t timeSpanCnt = 1;
    // Variable pour la position du digit � modifier de l'intervalle dans le mode config
    // Ex pour interevalle de  12:34
    // 0 => 4
    // 1 => 3
    // 2 => 2
    // 3 => 1
    uint8_t timeSpanDigit = 0;
    // Variable pour choisir la valeur � ajouter ou enlever de l'intervalle 
    // en fonction du digit choisi
    uint16_t timeSpanIncrDecrValue;
    // Variable pour la valeur minimum de l'intervalle en fonction
    // du digit s�lectionn�
    uint16_t timeSpanMinVal;
    // Variable pour la valeur maximum de l'intervalle en focntion 
    // du digit s�lectionnn�
    uint16_t timeSpanMaxVal;
    // Ex si timeSpandigit = 0 :
    // min =   1    =>    (0 + 1s)
    // max = 899    =>    (900 - 1s)
    // Ex si timeSpandigit = 1 :
    // min =  10    =>    (0 + 10s)
    // max = 890    =>    (900 - 10s)
    
    
    //                         LCD
    //=====================================================
    // structure pour sauvegarder la valeur des r�gistres utilis� pour l'�cran LCD
    static s_EADOGS_REG_VAL RegVal;
    RegVal.displayControl = DISPLAY_CONTROL;
    RegVal.extendedFunctionSet = EXTENDED_FUNCTION_SET;
    RegVal.functionSet_RE0 = FUNCTIONSET_RE0;
    RegVal.functionSet_RE1 = FUNCTIONSET_RE1;
    RegVal.power_Display_Contrast = POWER_DISPLAY_CONTRAST;
    
    static uint8_t Menu = 0;
    static uint8_t backLightLvl = 10;
    
    //               Flags
    //=====================================================
    // Point de d�part
    static bool startPointSet = false;
    // Point d'arriv�
    static bool endPointSet = false;
    bool SDCardmounted = false;
    bool waitForCmd = false; 
    //               USB
    //=====================================================
    // Connexion � l'application du PC
    static bool USB_Connected = false;
    // Buffer de r�ception
    static uint8_t USB_RxBuffer[20];
//    static uint8_t USB_TxBuffer[512];
    static uint8_t KeepAliveCnt = 0;
    
    
    //               Boutons
    //=====================================================
    // structures pour l'antirebond
    static S_SwitchDescriptor DescriptBtnMode;
    static S_SwitchDescriptor DescriptBtnGPS;
    static S_SwitchDescriptor DescriptBtnStartStop;
    
    //    GNSS
    //=====================================================
    static minmea_messages Messages;
	static s_CoordinatesParse Latitude;
	static s_CoordinatesParse Longitude;
//    static char line[MINMEA_MAX_SENTENCE_LENGTH] = {0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0,
//    0,0,0,0,0,0,0,0,0,0};
    
    // Compteur pour le clignotement des LEDs
    static uint8_t blinkCnt = 0;
    
    
//    SPI_DoTasks();
    
    /* Check the application's current state. */
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // D�marrage des timers 
            DRV_TMR0_Start();  // Pour le programme principale
            DRV_TMR1_Start();  // Pour les modules OCs
            // D�marrage des OCs
            DRV_OC0_PulseWidthSet(4999); // pour 100%
            DRV_OC0_Start();   // Pour le backlight de l'�cran LCD
//            DRV_OC1_Start();   // Pour le Buzzer 
            // Initialisation du ADC
            InitADC10();
            ReadAllAnalogValues();
            ReadBattery(&V_Bat);
//            V_Bat = ReadAllAnalogValues();
            // Initialisation des FIFOS pour les donn�es re�us par le r�cepteur GPS
            // et les donn�es � envoyer au PC
            InitFifoComm();
            
//            
            // Initialisation de l'�cran LCD
            if(SPI_GetState() == SPI_STATE_IDLE)
            {
                LCD_EADOGS_Init(&RegVal, BUS_8BITS, 4, BOTTOM_VIEW, false, true);
                LCD_BlinkCursor(&RegVal, true);
            }
           
            
            // Initialisation des antirebonds sur les boutons 
            DebounceInit(&DescriptBtnMode);
            DebounceInit(&DescriptBtnGPS);
            DebounceInit(&DescriptBtnStartStop);
            // Reset buffer de datas du r�cepteur GNSS
            ResetBuffer(GnssData);
            
            // Passer dans l'�tat d'attente
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
            // Gestion du nombre d'appuis sur les boutons ave
            // l'antirebonds
            //=========================
            // Bouton mode config
            ScanButtons(&DescriptBtnMode, BtnModeStateGet());
            // Bouton GPS
            ScanButtons(&DescriptBtnGPS, BtnGPSStateGet());
            // Bouton Start / Stop
            ScanButtons(&DescriptBtnStartStop, BtnStartStopStateGet()); 
                
            // Compteur pour le clognotement des LEDs
            blinkCnt = (blinkCnt + 1) %  BLINK_PER;

            //========================================================
            // Mesure du niveau de la batterie tous les 3s
            //========================================================
            ReadBattery(&V_Bat);
            // Affichage dans le LCD du niveau de batterie
//            if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
//            {
//                LCD_EADOGS_GoTo(&RegVal, 1, 1);
//                LCD_Printf("Niveau Batterie     %1.2f",V_Bat);
//                EnablePrint = false;
//            }
            //========================================================
            // Affichage sur la LEDs du niveau de batterie
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
                else if(blinkCnt == (BLINK_PER-1))
                {
                    BatLvl_ROff();
                }
                BatLvl_GOff();
                BatLvl_BOff();
            }
            //=========================================================
            // Detection de la pr�sence de la carte SD
            //=========================================================
            // Si carte SD pas inser�
            // Allumer la LED rouge du voyant SDCard
            if(SDCard_DetectStateGet() == false)
            {
                SDCard_ROn();
                SDCardmounted = false;
            }
            // si non l'�teindre
            else
            {
                SDCard_ROff();
                SDCardmounted = true;
            }
            
            //========================================================
            //                       Machine d'�tat
            //========================================================
            switch(appData.tracker_State)
            {
                //=====================================================
                //                      Idle
                //=====================================================
                case INIT:
                    
                    if(Cnt == 1)
                    {
                        if(SPI_GetState() == SPI_STATE_IDLE)
                        {
                            LCD_EADOGS_GoTo(&RegVal, 1,1);
                            LCD_Printf("2308 TrackerGPS");
                            LCD_EADOGS_GoTo(&RegVal, 1,2);
                            LCD_Printf("Einar Farinas");
                        }
                    }
                    if(Cnt == 250)
                    {
//                        LCD_EADOGS_GoHome(&RegVal);
                        EnablePrint = true;
                        appData.tracker_State = IDLE;
                    }
                    Cnt++;
                    break;
                //=====================================================
                //                      Idle
                //=====================================================
                case IDLE:
                    if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                    {
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
                        LCD_Printf("      IDLE     ");
                        EnablePrint = false;
                    }
                    // Si appui 1 fois sur le bouton GPS
                    if(DescriptBtnGPS.nbTouch == 1)
                    {
//                        DescriptBtnGPS.nbTouch = 0;
                        // Passer dans l'�tat d'attente de connexion aux satellites
                        EnablePrint = true;
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
                    if(DescriptBtnMode.HoldCnt >= 300)
                    {
                        // Passer dans l'�tat de configuration
                        EnablePrint = true;
                        appData.tracker_State = CONFIG;
                    } 
                    // Affichage �tat idle
                    
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
                    
                    
                    // Si le bouton Mode est appuy�e pendant 3s
                    break;
                //=====================================================
                //          Attente de connection aux satellites
                //=====================================================
                case WAIT_FOR_CONNECTION:
                    if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                    {
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal,1,1);
                        LCD_Printf("Connecting...");
                        LCD_EADOGS_GoTo(&RegVal,1,2);
                        LCD_Printf("Sat=%2d", Messages.gsv.total_sats);
                        LCD_EADOGS_GoTo(&RegVal,1,3);
                        LCD_Printf("CNO=%2d", Messages.gsv.sats[0].snr);
//                        LCD_EADOGS_GoTo(&RegVal,1,3);
//                        LCD_Printf("N�36...");
//                        LCD_EADOGS_GoTo(&RegVal,1,4);
//                        LCD_Printf("E�06...");
                        EnablePrint = false;
                    }
                    // Attendre la r�ecpetion d'une cordonn�e
                    // clignotement LED GPS
                    if(blinkCnt == (BLINK_PER / 2))
                    {
                        GPS_GOn();
                    }
                    else if(blinkCnt == (BLINK_PER -1))
                    {
                        GPS_GOff();
                    }
//                    dataSize = ScanGnssCmd(MINMEA_SENTENCE_GSV, GnssData);
                    dataSize = GetGnssCmd(GnssData);
                    if(dataSize != 0)
                    {
                        if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_GSV)
                        {
                            minmea_parse_gsv(&Messages.gsv, (char*)GnssData);
                        }
                        else if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_RMC)
                        {
                            minmea_parse_rmc(&Messages.rmc, (char*)GnssData);
                            if((Messages.rmc.latitude.value != 0) && (Messages.rmc.longitude.value != 0))
                            {
                                EnablePrint = true;
                                appData.tracker_State = READY;
                                GPS_GOn();
                            }
                        }
                        
                        Cnt = (Cnt + 1) % 50;
                        if(Cnt == 0)
                        {
                            EnablePrint = true;
                        }
//                        SetBuffer(GnssData, dataSize);
                    }
                    ResetBuffer(GnssData);
//                    dataSize = ScanGnssCmd(MINMEA_SENTENCE_GSV, GnssData);
//                    if(dataSize != 0)
//                    {
////                        SendUsbMessage(GnssData, dataSize);
//                        minmea_parse_gsv(&Messages.gsv, GnssData);
////                        if(Messages.gsv.total_sats < 0)
////                        {
////                            Messages.gsv.total_sats = 0;
////                        }
////                        if(Messages.gsv.sats[0].snr < 0)
////                        {
////                            Messages.gsv.sats[0].snr = 0;
////                        }
//                        if(Cnt == 0)
//                        {
//                            EnablePrint = true;
//                        }
////                        SetBuffer(GnssData, dataSize);
//                    }
                    
                    if(DescriptBtnGPS.nbTouch == 2)
                    {
                        DescriptBtnGPS.nbTouch = 0;
                        // Passer dans l'�tat d'attente de connexion aux satellites
                        appData.tracker_State = READY;
                        EnablePrint = true;
                    }
                    break;
                //=====================================================
                // Connexion aux satellites r�ussie
                // syst�me pr�t � acquerir et sauvegarder des coordonn�es
                //=====================================================
                case READY:
                    if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                    {
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 3, 2);
                        LCD_Printf("READY");
                        EnablePrint = false;
                    }
                    // Si appui 1 fois sur le bouton GPS et le point de d�part n'a pas �t� sauvegard�
                    if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == false))
                    {
                        // Retour � l'�tat idle
                        appData.tracker_State = IDLE;
                        DescriptBtnGPS.nbTouch = 0;
                        EnablePrint = true;
//                        break;
                    }
                    // Si appui 1 fois sur le bouton GPS et et point de d�part a �t� sauvegard�
                    else if((DescriptBtnGPS.nbTouch == 1) && (startPointSet == true))
                    {
                        // Retour � l'�tat TRACKING pour l'acquisition des coordonn�es
                        appData.tracker_State = TRACKING;
                        // Reset flag
                        startPointSet = false;
//                        DescriptBtnGPS.nbTouch = 0;
//                        break;
                    }
                    // Si appui sur le bouton Start
                    else if((DescriptBtnStartStop.nbTouch == 1) && (startPointSet == false))
                    {
                        // Set le flag du point de d�part
                        startPointSet = true;
                        DescriptBtnStartStop.nbTouch = 0;
                        // Savegarde la coordonn�e de d�part 
//                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    break;
                //=====================================================
                // Acquisition et savegarde de coordonn�es selon l'intervalle de temps
                //===================================================== 
                case TRACKING:
                    if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                    {
                        LCD_EADOGS_Clear();
                        LCD_EADOGS_GoTo(&RegVal, 1, 1);
                        LCD_Printf("Tracking");
                        LCD_EADOGS_GoTo(&RegVal, 1, 2);
                        LCD_Printf("%02d:%02d %02d/%02d/%4d", Messages.rmc.time.hours, Messages.rmc.time.minutes, Messages.rmc.date.day, Messages.rmc.date.month, Messages.rmc.date.year);
                        LCD_EADOGS_GoTo(&RegVal, 1, 3);
                        LCD_Printf("N%02d�%02d'%02d\"", Latitude.degrees, Latitude.min, Latitude.sec);
                        LCD_EADOGS_GoTo(&RegVal, 1, 4);
                        LCD_Printf("N%02d�%02d'%02d\"", Longitude.degrees, Longitude.min, Longitude.sec);
                        EnablePrint = false;
                    }
                    Cnt = (Cnt + 1) % 100;
                    if(Cnt == 0)
                    {
                        timeSpanCnt = (timeSpanCnt + 1) % timeSpan;
                    }
                    // Si temps d'intervalle �coul�
                    if(timeSpanCnt == (timeSpan - 1))
                    {
                        dataSize = GetGnssCmd(GnssData);
                        if(dataSize != 0)
                        {
                            if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_GSV)
                            {
                                minmea_parse_gsv(&Messages.gsv, (char*)GnssData);
                            }
                            else if((minmea_sentence_id((char*)GnssData, true)) == MINMEA_SENTENCE_RMC)
                            {
                                minmea_parse_rmc(&Messages.rmc, (char*)GnssData);
                            }
                            if(Messages.rmc.valid == true)
                            {
                                EnablePrint = true;
								// Conversion des coordonn�es
								Latitude.degrees = (uint8_t) Messages.rmc.latitude.value / 100;
								Latitude.min = (uint8_t) Messages.rmc.latitude.value - (Latitude.degrees * 100);
								Latitude.sec = ((Messages.rmc.latitude.value - (Latitude.degrees * 100.0)) - (float)Latitude.min) * 60;
								
								Longitude.degrees = (uint8_t) Messages.rmc.longitude.value / 100;
								Longitude.min = (uint8_t) Messages.rmc.longitude.value - (Latitude.degrees * 100);
								Longitude.sec = ((Messages.rmc.longitude.value - (Latitude.degrees * 100.0)) - (float)Latitude.min) * 60;

                            }
    //                        SetBuffer(GnssData, dataSize);
                        }
                        // Reset compteur
//                        timeSpanCnt = 0;
                        // Savegarde la coordonn�e de d�part 
//                        appData.tracker_State = SAVE_COORDINATES;
                    }
                    ResetBuffer(GnssData);
                    // Si appui une premi�re fois sur le bouton start/stop
                    // Set flag pour indiquer le point d'arriv�
                    if((DescriptBtnStartStop.nbTouch == 1)  && (endPointSet == false))
                    {
                        // Set flag du point d'arriv�e
                        endPointSet = true;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    // Si flag de point d'arriv� set et appui 2 fois sur 
                    // bouton GPS
                    // Confirmer le point d'arriv�
                    else if((DescriptBtnStartStop.nbTouch == 2) && (endPointSet == true))
                    {
                        // Savegarde la coordonn�e de d�part 
                        appData.tracker_State = SAVE_COORDINATES;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    else if(DescriptBtnStartStop.nbTouch != 0)
                    {
                        // Reset flag du point d'arriv�e
                        endPointSet = false;
                        // Reset compteur
                        DescriptBtnStartStop.nbTouch = 0;
                    }
                    break;
                //=====================================================
                // Savegarde coordonn�es dans la carte SD
                //=====================================================
                case SAVE_COORDINATES:
//                    LCD_Printf("SAVECOORD");
                    // Si sauvegarde du point d'arrvi� ou de d�part
                    if((startPointSet == true) || (endPointSet == true))
                    {
                        // Reset flag d'arriv�
                        endPointSet = false;
                        // Passer dans l'�tat READY
                        appData.tracker_State = READY;
                    }
                    // Si sauvegarde du point selon intervalle de temps
                    else if((startPointSet == false) && (endPointSet == false))
                    {
                        // Retourner dans l'�tat TRACKING
                        appData.tracker_State = TRACKING;
                    }
                    break;
                //=====================================================
                // Lors de la connection par USB au PC
                //=====================================================
                // Attente de demande de lecture des donn�es sur la carte SD
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
                // Lecture et r�cup�ration des donn�es sur la carte SD
                case USB_READ_DATA:
//                    LCD_Printf("USB_READ");
                    // Attente que la lecture du fichier soit finie
                    if(GetSDCardReadFlag() == false)
                    {
                        appData.tracker_State = USB_SEND_DATA;
                    }
                    break;
                // Envoi des donn�es r�cup�r�es par USB au PC
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
                // Pour r�gler l'intervalle de temps ou autres param�tres
                case CONFIG:
                    
                    switch(Menu)
                    {
                        case 0:
                            if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,2,1);
                                LCD_Printf("Mode config");
                                LCD_EADOGS_GoTo(&RegVal,3,2);
                                LCD_Printf(">Intervalle<");
                                LCD_EADOGS_GoTo(&RegVal,4,3);
                                LCD_Printf("Backlight");
                                EnablePrint = false;
                            }
                            if(DescriptBtnMode.nbTouch == 1)
                            {
                                Menu = 2;
                                EnablePrint = true;
                                DescriptBtnMode.nbTouch = 0;
                            }
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                Menu = 1;
                                EnablePrint = true;
                                DescriptBtnStartStop.nbTouch = 0;
                            }
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                Menu = 1;
                                EnablePrint = true;
                                DescriptBtnGPS.nbTouch = 0;
                            }
                            if(DescriptBtnMode.HoldCnt == 300)
                            {
                                EnablePrint = true;
                                DescriptBtnMode.HoldCnt = 0;
                                appData.tracker_State = IDLE;
                            }
                            break;
                        case 1:
                            if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,2,1);
                                LCD_Printf("Mode config");
                                LCD_EADOGS_GoTo(&RegVal,4,2);
                                LCD_Printf(">Backlight<");
                                LCD_EADOGS_GoTo(&RegVal,3,3);
                                LCD_Printf("Intervalle");
                                EnablePrint = false;
                            }
                            if(DescriptBtnMode.nbTouch == 1)
                            {
                                Menu = 3;
                                EnablePrint = true;
                                DescriptBtnMode.nbTouch = 0;
                            }
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                Menu = 0;
                                EnablePrint = true;
                                DescriptBtnStartStop.nbTouch = 0;
                            }
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                Menu = 0;
                                EnablePrint = true;
                                DescriptBtnGPS.nbTouch = 0;
                            }
                            if(DescriptBtnMode.HoldCnt == 300)
                            {
                                EnablePrint = true;
                                DescriptBtnMode.HoldCnt = 0;
                                appData.tracker_State = IDLE;
                            }
                            break;
                        case 2:
                            if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,1,2);
                                LCD_Printf("Mode config");
                                LCD_EADOGS_GoTo(&RegVal,1,3);
                                LCD_Printf("Intervalle : ");
                                LCD_EADOGS_GoTo(&RegVal,1,4);
                                LCD_Printf("%d : %d", timeSpan/60, timeSpan );
                                EnablePrint = false;
                            }
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
                                // Secondes unit�es
                                case 0:
                                    timeSpanIncrDecrValue = 1;
                                    break;
                                // Secondes dixi�mes
                                case 1:
                                    timeSpanIncrDecrValue = 10;
                                    break;
                                // Minutes unit�es
                                case 2:
                                    timeSpanIncrDecrValue = 60;
                                    break;
                                // Minutes dixi�mes
                                case 3:
                                    timeSpanIncrDecrValue = 600;
                                    break;
                                default:
                                    break;
                            }
                            // Set les valeurs min et max pour pouvoir
                            // incr�meter ou d�cr�menter la valeur
                            timeSpanMinVal = (TIME_SPAN_MIN + timeSpanIncrDecrValue);
                            timeSpanMaxVal = (TIME_SPAN_MAX - timeSpanIncrDecrValue);

                            // Incr�mentation de la valeur du digit
                            // avec le bouton StartStop
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                if(timeSpan <= timeSpanMaxVal)
                                {
                                    timeSpan += timeSpanIncrDecrValue;
                                    EnablePrint = true;
                                }
                                DescriptBtnStartStop.nbTouch = 0;
                            }
                            else if(DescriptBtnStartStop.nbTouch != 0)
                            {
                                DescriptBtnStartStop.nbTouch = 0;
                            }
                            // D�cr�mentation de la valeur du digit
                            // avec le boutons GPS
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                if(timeSpan >= timeSpanMinVal)
                                {
                                    timeSpan -= timeSpanIncrDecrValue;
                                    EnablePrint = true;
                                }
        //                        DescriptBtnGPS.nbTouch = 0;
                            }
                            else if(DescriptBtnGPS.nbTouch != 0)
                            {
        //                        DescriptBtnGPS.nbTouch = 0;
                            }
                            if(DescriptBtnMode.nbTouchCnt == 2)
                            {
                                EnablePrint = true;
                                Menu = 0;
                                DescriptBtnMode.nbTouchCnt = 0;
                            }
                            break;
                        case 3:
                            if((SPI_GetState() == SPI_STATE_IDLE) && (EnablePrint == true))
                            {
                                LCD_EADOGS_Clear();
                                LCD_EADOGS_GoTo(&RegVal,1,2);
                                LCD_Printf("Mode config");
                                DRV_OC0_PulseWidthSet(BL_100 * backLightLvl / 100);
                                LCD_EADOGS_GoTo(&RegVal,1,3);
                                LCD_Printf("Backlight : %d %%", backLightLvl);
                                EnablePrint = false;
                            }
                            if(DescriptBtnStartStop.nbTouch == 1)
                            {
                                if(backLightLvl < 100)
                                {
                                    backLightLvl++;
                                    EnablePrint = true;
                                    DescriptBtnStartStop.nbTouch = 0;
                                }
                            }
                            else if(DescriptBtnStartStop.HoldCnt >= 100)
                            {
                                if(backLightLvl < 100)
                                {
                                    backLightLvl++;
                                    EnablePrint = true;
                                }
                            }
                            if(DescriptBtnGPS.nbTouch == 1)
                            {
                                if(backLightLvl > 0)
                                {
                                    backLightLvl--;
                                    EnablePrint = true;
                                    DescriptBtnGPS.nbTouch = 0;
                                }
                            }
                            else if(DescriptBtnGPS.HoldCnt >= 100)
                            {
                                if(backLightLvl > 0)
                                {
                                    backLightLvl--;
                                    EnablePrint = true;
                                }
                            }
                            if(DescriptBtnMode.nbTouchCnt == 2)
                            {
                                EnablePrint = true;
                                Menu = 0;
                                DescriptBtnMode.nbTouchCnt = 0;
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
            DescriptBtnGPS.nbTouch = 0;
            DescriptBtnStartStop.nbTouch = 0;  
            DescriptBtnMode.nbTouch = 0;
            
//            GetGnssMessage(GnssData, 50);
//            
//            
//            
            
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
    appData.state = newState; // mise � jour d'�tat
}

// Utiliser pour test
void Timer1_CallBack(void)
{
    // Compteur pour pour le changment de LEDS
    static uint8_t Cnt = 0;
    static uint8_t SelectLed = 0;
    
    
    
    // Comtpteur de 0 � 100 pour chager de LED chaque 1 seconde (T timer = 10 ms) 
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
    if((DebounceIsReleased(DescriptButton)) && (DebounceIsPressed(DescriptButton)) && (DescriptButton->HoldCnt != 0))
    {             
        // Reset compteur de timeout
        DescriptButton->TimeoutCnt = 0;

        if(DescriptButton->nbTouchCnt < NB_PRESS_BTN_MAX)
        {
            DescriptButton->nbTouchCnt++;
        }
        // 
        DebounceClearPressed(DescriptButton);
        DebounceClearReleased(DescriptButton);
    }
    if(DebounceGetInput(DescriptButton) == 1)
    {
        DescriptButton->HoldCnt++;
        if(DescriptButton->HoldCnt >= 50)
        {
            DescriptButton->nbTouchCnt = 0;
        }
    }
    else
    {
        DescriptButton->HoldCnt = 0;
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

void ReadBattery(float *VBat)
{
    // Flag pour activer la mesure de tension de la batterie
    static uint16_t V_Bat_Mes_Cnt = 0;
    
    if(V_Bat_Mes_Cnt == 0)
    {
        EN_ReadBATOn();
    }
    if(V_Bat_Mes_Cnt == 1)
    {
        *VBat = ReadAllAnalogValues();
//        EnablePrint = true;
        EN_ReadBATOff();
    }
    V_Bat_Mes_Cnt = (V_Bat_Mes_Cnt + 1) % BAT_TIME;
}
/*******************************************************************************
 End of File
 */
