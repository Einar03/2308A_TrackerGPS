#ifndef __MC32CORETIMER_H
#define __MC32CORETIMER_H
/*--------------------------------------------------------*/
//      Mc32CoreTimer.h
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

#include <stdint.h>
#include <cp0defs.h>

/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/

void OpenCoreTimer( uint32_t compare);
uint32_t ReadCoreTimer(void);
void UpdateCoreTimer( uint32_t period);
void WriteCoreTimer( uint32_t val);

#endif
