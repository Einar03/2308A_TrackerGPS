/*--------------------------------------------------------*/
//  Mc32CoreTimer.h
/*--------------------------------------------------------*/
//	Description :	Realisation des fonctions de gestion du
//					Core Timer suite à suppression de peripheral\timer.h
//
//	Auteur 		: 	C. HUBER
//
//	Version		:	V1.0
//	Compilateur	:	XC32 depuis 1.4
//
/*--------------------------------------------------------*/


#include "Mc32CoreTimer.h"
#include <xc.h>


/*--------------------------------------------------------*/
// Fonction OpenCoreTimer
/*--------------------------------------------------------*/
// Cette fonction charge la valeur de comparaison et met à 0 le compteur


void OpenCoreTimer( uint32_t compare)
{
     _CP0_SET_COMPARE(compare);
     _CP0_SET_COUNT(0);
}


/*--------------------------------------------------------*/
// Fonction ReadCoreTimer
/*--------------------------------------------------------*/
// fourni la valeur du Core Timer ( CP0 registre COUNT)

uint32_t ReadCoreTimer(void)
{
    return ( _CP0_GET_COUNT() );
}

/*--------------------------------------------------------*/
// Fonction UpdateCoreTimer
/*--------------------------------------------------------*/
// Cette fonction ajoute l'argument periode à la valeur du
// registre COMPARE du CP0

void UpdateCoreTimer( uint32_t period)
{
    uint32_t NewCompare;

    NewCompare = _CP0_GET_COMPARE() + period;
    _CP0_SET_COMPARE(NewCompare);
}

/*--------------------------------------------------------*/
// Fonction WriteCoreTimer
/*--------------------------------------------------------*/
// Impose une valeur au registre COUNT

void WriteCoreTimer( uint32_t val)
{
    _CP0_SET_COUNT(val);
}

