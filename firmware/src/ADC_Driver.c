/* 
 * File:   ADC_Driver.c
 * 
 * Author: Jonathan CHAFLA
 * 
 * Project: 2220_AlimStepDown
 * 
 * Description: Fichier permetant la gestion des ADCs du projet 2220
 *  Adaptée pour le proje 2308A trackerGPS
 * 
 * Version: V1
 *
 * Created on 3. mai 2023
 */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */

// ------------ Includes ------------ //
#include "ADC_Driver.h"
#include "system_config.h"
#include "peripheral/adc/plib_adc.h"



/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

// ------------ Variables globales ------------ //


// ------------ Prototypes ------------ //


//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : InitADC10
// In           : -
// Out          : -
// Description  : fonction d'initialisation des ADCs
// Version      : V1
// date         : 08.09.2022
// Auteur       : Jonathan CHAFLA 
//----------------------------------------------------------------------------------------------------------------------------------------
void InitADC10(void)
{

    PLIB_ADC_InputScanMaskAdd(ADC_ID_1, ADC_AN_SCAN_ADDRES);
    // Configure l'ADC en mode alterné
    PLIB_ADC_ResultFormatSelect(ADC_ID_1, ADC_RESULT_FORMAT_INTEGER_16BIT);
    //Mode alterné
    PLIB_ADC_ResultBufferModeSelect(ADC_ID_1, ADC_BUFFER_MODE_TWO_8WORD_BUFFERS);
    //mode multiplexage
    PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA);
    
    //la lecture des ADC est cadensée par le timer interne
    PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
    PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS);
    PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 0x1F);
    PLIB_ADC_ConversionClockSet(ADC_ID_1, SYS_CLK_FREQ, 32);
    
    //ADC fait 3 mesures par interruption (car 3 canaux utillisés) 
    PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_1SAMPLE_PER_INTERRUPT);
    //active le scan en mode multiplexage des entrées AN
    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);
    
    // Enable the ADC module
    PLIB_ADC_Enable(ADC_ID_1);
}

//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : ReadAllADC
// In           : -
// Out          : structure avec les valeurs brutes des ADCs
// Description  : fonction de lecture de tout les ADCs utiles à ce projet (AN3, AN4, AN5, AN6, AN7 et AN9) 
// Version      : V1
// date         : 03.05.2023
// Auteur       : Jonathan CHAFLA
//----------------------------------------------------------------------------------------------------------------------------------------
S_ADCResults ReadAllADC()
{
    //structure de valeurs brutes des ADCs
    volatile S_ADCResults rawResult;
    ADC_RESULT_BUF_STATUS BufStatus;    
//    S_ADCResults meanedResult;
   
    //stop sample/convert
    PLIB_ADC_SampleAutoStartDisable(ADC_ID_1);
    
    // traitement avec buffer alterné
    BufStatus = PLIB_ADC_ResultBufferStatusGet(ADC_ID_1);
    
    if (BufStatus == ADC_FILLING_BUF_0TO7) {
        rawResult.AN0  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0);
//        rawResult.AN1  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 1);
//        rawResult.AN4  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 2);    
    }
    else
    {
        rawResult.AN0  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 8);
//        rawResult.AN1  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 9);
//        rawResult.AN4  = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 10); 
    }
    
    // Retablit Auto start sampling
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
        
    return rawResult;
}


//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : ReadAllAnalogValues
// In           : -
// Out          : valeurs converties de tout les ADCs utillisés (grandeures réelles des tension, courants ou température)
//                les tensions sont retournées en mV, les courants en mA et température en degré celcius  
//
// Description  : Permet de lire les ADCs utiles à ce projet (AN2, AN4, AN6, AN7 et AN8), et convertis les valeurs brutes en 
//                grandeurs réelles (tension, courants ou tempéraure)
// Version      : V1
// date         : 03.05.2023
// Auteur       : Jonathan CHAFLA
//----------------------------------------------------------------------------------------------------------------------------------------
float ReadAllAnalogValues(void)
{
    float Uin_A1;// Uout_A1;
    float Uin_A2;//, Uout_A2, Iout_A2;
    int Uin_P1;//, Uout_P1;
    float Uin_P2;//, Uout_P2, Iout_Val1;
    float Uin_Res;//, Uout_Res, Iout_Res;
 
    //structure de valeurs brutes des ADCs
    S_ADCResults ADCRawValues;
    //structure des grandeurs réelles
//    float realValues;
    
    //lecture des valeurs brutes des ADCs
    ADCRawValues = ReadAllADC();
    
    /*----------------------------------------------------------*/
    //Gestion Uin 
    Uin_A1 = (float)ADCRawValues.AN0 / (float)ADC_SIZE;
    Uin_A2 = (float)Uin_A1 * (float)VREF_UC_VAL;
    Uin_P1 =  Vin_LOWER_RESISTOR_VAL + Vin_UPPER_RESISTOR_VAL;
    Uin_P2 = (float) Vin_LOWER_RESISTOR_VAL / (float)Uin_P1;
    Uin_Res = (float) Uin_A2 / (float) Uin_P2;
    
//    realValues.Uin = Uin_Res;
    return Uin_Res;
    
//    /*----------------------------------------------------------*/
//    //Gestion Uout
//    Uout_A1 = (float)ADCRawValues.AN0 / (float)ADC_SIZE;
//    Uout_A2 = (float)Uout_A1 * (float)VREF_UC_VAL;
//    Uout_P1 =  Vout_UPPER_RESISTOR_VAL + Vout_LOWER_RESISTOR_VAL;
//    Uout_P2 = (float) Vout_LOWER_RESISTOR_VAL / (float)Uout_P1;
//    Uout_Res = (float) Uout_A2 / (float) Uout_P2;
//    
//    realValues.Uout = Uout_Res;
    
    
//    /*----------------------------------------------------------*/
//    //Conversion pour Iout
//    Iout_A2 = (float)ADCRawValues.AN1 / (float)ADC_SIZE;
//    Iout_Val1 = Iout_A2 * (float)VREF_UC_VAL;
//    Iout_Res = (Iout_Val1 / VMAX) * IMAX;
//    
//    realValues.Iout = Iout_Res;
    
    //retourne tout return realValues;es les valeurs 
//    return realValues;
}
