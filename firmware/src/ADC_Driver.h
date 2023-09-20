/* 
 * File:   ADC_Driver.h
 * 
 * Author: Jonathan CHAFLA
 * 
 * Project: 2220_AlimStepDown
 * 
 * Description: Fichier permetant la gestion des ADCs du projet 2220_AlimStepDown
 * 
 * Version: V1
 *
 * Created on 3. mai 2023
 */

#ifndef ADC_DRIVER_H
#define	ADC_DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

    
// ------------ Includes ------------ //
#include <stdint.h>

// ------------ Constantes ------------ //
    
#define ADC_AN_SCAN_ADDRES      0x0013     //addition des addresses des AN à lire (AN0, AN1, AN4)      
    
#define ADC_SIZE                1023        //taille de l'ADC (10 bits)
    
#define VREF_UC_VAL             3.3         //tension de la référence de tension de l'uC
    
#define VMAX                    3.0
#define IMAX                    5.0
    
#define Vin_UPPER_RESISTOR_VAL  3000      //valeur en ohm de la résistance du haut du diviseur de tension pour Uin (réduction de tension pour uC)
#define Vin_LOWER_RESISTOR_VAL  7500        //valeur en ohm de la résistance du bas du diviseur de tension pour Uin (réduction de tension pour uC)
    
#define Vout_UPPER_RESISTOR_VAL 2000      //valeur en ohm de la résistance du haut du diviseur de tension pour Uout (réduction de tension pour uC)
#define Vout_LOWER_RESISTOR_VAL 3000       //valeur en ohm de la résistance du bas du diviseur de tension pour Uout (réduction de tension pour uC)
    
#define NB_ADC_CHANEL           3           //nbr de channels de l'ADC
#define SLIDING_MEAN_SAMPLES    10          // nbr d'echantillons pour la moyenne glisante

#define CHANNEL_0               0          // chanel 0 ADC
#define CHANNEL_1               1          // chanel 1 ADC
#define CHANNEL_4               4          // chanel 4 ADC

    
// ------------ Variables globales ------------ //
typedef struct 
{
    float Uout;          // Tension de sortie
    float Iout;          // courant de sortie
    float Uin;           //Tension d'entrée   
} S_ADCTransformedResults;

typedef struct 
{
    uint16_t AN0;       // Valeur brute ADC de la tension de sortie mesurée
//    uint16_t AN1;       // Valeur brute ADC du courant de sortie mesurée
//    uint16_t AN4;       // Valeur brute ADC de la tension d'entrée mesurée 
} S_ADCResults;


// ------------ Prototypes ------------ //

//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : InitADC10
// In           : -
// Out          : -
// Description  : fonction d'initialisation des ADCs
// Version      : V1
// date         : 03.05.2023
// Auteur       : Jonathan CHAFLA
//----------------------------------------------------------------------------------------------------------------------------------------
void InitADC10(void);
    
    
//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : ReadAllADC
// In           : -
// Out          : structure avec les valeurs brutes des ADCs
// Description  : fonction de lecture de tout les ADCs utiles à ce projet (AN2, AN4, AN6, AN7, AN8)
// Version      : V1
// date         : 03.05.2023
// Auteur       : Jonathan CHAFLA
//----------------------------------------------------------------------------------------------------------------------------------------
S_ADCResults ReadAllADC(void);  


//----------------------------------------------------------------------------------------------------------------------------------------
// Fonction     : ReadAllAnalogValues
// In           : -
// Out          : valeurs converties de tout les ADCs utillisés (grandeures réelles des tension, courants ou température)
// Description  : Permet de lire les ADCs utiles à ce projet (AN2, AN4, AN6, AN7, AN8), et convertis les valeurs brutes en 
//                grandeurs réelles (tension, courants ou tempéraure)
// Version      : V1
// date         : 03.05.2023
// Auteur       : Jonathan CHAFLA
//----------------------------------------------------------------------------------------------------------------------------------------
float ReadAllAnalogValues(void);


  
#ifdef	__cplusplus
}
#endif

#endif	/* ADC_DRIVER_H */

