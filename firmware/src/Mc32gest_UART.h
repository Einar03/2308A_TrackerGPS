#ifndef Mc32Gest_UART_H
#define Mc32Gest_UART_H
/*--------------------------------------------------------*/
// Mc32Gest_RS232.h
/*--------------------------------------------------------*/
//	Description :	emission et reception spécialisée
//			        pour TP2 2016-2017
//
//	Auteur 		: 	C. HUBER
//
//	Version		:	V1.3
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include <stdint.h>
#include "GesFifoTh32.h"

// ======= DEFINES ======
#define MESS_SIZE 5

/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/

// prototypes des fonctions
void InitFifoComm(void);
void GetGnssMessage(uint8_t *pData, uint8_t NbBytesToGet);
void SendGnssMessage(uint8_t *pData, uint8_t nbBytesToSend);
void GetUsbMessage(uint8_t *pData, uint8_t NbBytesToGet);
void SendUsbMessage(uint8_t *pData, uint8_t nbBytesToSend);
uint8_t GetGnssCmd(uint8_t *pData);

// Descripteur des fifos
extern S_fifo descrFifoRX;
extern S_fifo descrFifoTX;

#endif
