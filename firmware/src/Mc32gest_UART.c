// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'émission et de réception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoyé réponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_UART.h"
#include "stdbool.h"
#include "minmea.h"


//typedef union {
//        uint16_t val;
//        struct {uint8_t lsb;
//                uint8_t msb;} shl;
//} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5

//// Structure décrivant le message
//typedef struct {
//    uint8_t Start;
//    int8_t  Speed;
//    int8_t  Angle;
//    uint8_t MsbCrc;
//    uint8_t LsbCrc;
//} StruMess;


//// Struct pour émission des messages
//StruMess TxMess;
//// Struct pour réception des messages
//StruMess RxMess;

// Declaration des FIFO pour réception et émission
#define FIFO_RX_SIZE 260
#define FIFO_TX_SIZE 260

// Pour le récepteur GNSS
int8_t GNSSfifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrGNSSFifoRX;

int8_t GNSSfifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrGNSSFifoTX;

// Pour la transmission par USB
int8_t USBfifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrUSBFifoRX;

int8_t USBfifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrUSBFifoTX;

// Initialisation de la communication sérielle
void InitFifoComm(void)
{    
    // Récepeteur GNSS
    // Initialisation du fifo de réception
    InitFifo ( &descrGNSSFifoRX, FIFO_RX_SIZE, GNSSfifoRX, 0 );
    // Initialisation du fifo d'émission
    InitFifo ( &descrGNSSFifoTX, FIFO_TX_SIZE, GNSSfifoTX, 0 );
    
    // USB
    // Initialisation du fifo de réception
    InitFifo ( &descrUSBFifoRX, FIFO_RX_SIZE, USBfifoRX, 0 );
    // Initialisation du fifo d'émission
    InitFifo ( &descrUSBFifoTX, FIFO_TX_SIZE, USBfifoTX, 0 );
} // InitComm

void GetGnssMessage(uint8_t *pData, uint8_t nbBytesToGet)
{
    uint8_t i = 0;
    uint8_t NbCharInFifo = 0;
    
    NbCharInFifo = GetReadSize(&descrGNSSFifoRX);
    if(NbCharInFifo >= nbBytesToGet)
    {
        for(i = 0; i < nbBytesToGet; i++)
        {
            GetCharFromFifo(&descrGNSSFifoRX, pData);
            pData++;
        }
//        return true;
    }
//    else
//    {
//        return false;
//    }
} // GetMessage

uint8_t GetGnssCmd(uint8_t *pData)
{
    uint16_t NbCharInFifo = 0;
    uint8_t start = 0;
    uint8_t nBData = 0;
    uint8_t endData_1 = 0;
    NbCharInFifo = GetReadSize(&descrGNSSFifoRX);
    
    if(NbCharInFifo >= 20)
    {
        GetCharFromFifo(&descrGNSSFifoRX, &start);
        if(start == '$')
        {
            *pData = '$';
            
            do
            {
                if(GetReadSize(&descrGNSSFifoRX) > 0)
                {
                    endData_1 = *pData;
                    pData++;
                    nBData++;
                    GetCharFromFifo(&descrGNSSFifoRX, pData);
                }
            }while(!((endData_1 == 0x0D) && (*pData == 0x0A)));
//            return nBData;
            nBData++;
            return nBData;
        }
        else
        {
//            return nBData;
            return nBData;
        }
    }
    else
    {
//        return nBData;
        return nBData;
    }
}
//bool ScanGnssCmd(enum minmea_sentence_id sentenceMsg, minmea_messages *frame, uint8_t *pData)
uint8_t ScanGnssCmd(enum minmea_sentence_id sentenceMsg, uint8_t *pData)
{
    // 
    uint8_t nbDatas;
    // Recupération de la commande réçue
    nbDatas = GetGnssCmd(pData);
    if(nbDatas != 0)
    {
        // Si la commande voulue a été récupérée
        if((minmea_sentence_id((char*)pData, true)) == sentenceMsg)
        {
//            switch(sentenceMsg)
//            {
//                case MINMEA_SENTENCE_GBS :
//                    break;
//                case MINMEA_SENTENCE_GGA :
//                    break;
//                case MINMEA_SENTENCE_GLL :
//                    break;
//                case MINMEA_SENTENCE_GSA :
//                    break;
//                case MINMEA_SENTENCE_GST :
//                    break;
//                case MINMEA_SENTENCE_GSV :
//                    break;
//                case MINMEA_SENTENCE_RMC :
//                    minmea_parse_rmc(frame, pData);
//                    break;
//                case MINMEA_SENTENCE_VTG :
//                    break;
//                case MINMEA_SENTENCE_ZDA :
//                    break;
//                default:
//                    break;
//            }
            return nbDatas;
        }
        else
        {
            return 0;
        }
        
    }
    else
    {
        return nbDatas;
    }
    
    
    
}        



// Fonction d'envoi des messages, appel cyclique
void SendGnssMessage(uint8_t *pData, uint8_t nbBytesToSend)
{
    uint8_t i = 0;
    int8_t FreeSize = 0;
    //selon spec. CCITT il faut initialiser la valeur du Crc16 à  0xFFFF
   
    // Traitement émission à  introduire ICI
    // Formatage message et remplissage fifo émission
    FreeSize = GetWriteSpace (&descrGNSSFifoTX);
    if (FreeSize >= nbBytesToSend)
    {
        for(i = 0; i < nbBytesToSend; i++)
        {
            // Dépose le message dans le fifo
            PutCharInFifo (&descrGNSSFifoTX, *pData);
            pData++;
        }
    }
    if (FreeSize > 0)
    {  
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
}



bool GetUsbMessage(uint8_t *pData, uint8_t nbBytesToGet)
{
    uint8_t i = 0;
    uint8_t start = 0;
    uint8_t NbCharInFifo = 0;
    
    NbCharInFifo = GetReadSize(&descrUSBFifoRX);
    if(NbCharInFifo >= nbBytesToGet)
    {
        GetCharFromFifo(&descrUSBFifoRX, &start);
        if(start == '!')
        {
            *pData = start;
            pData++;
            for(i = 0; i < (nbBytesToGet-1); i++)
            {
                GetCharFromFifo(&descrUSBFifoRX, pData);
                pData++;
            }
        }
        
        return true;
    }
    else
    {
        return false;
    }
} // GetMessage

// Fonction d'envoi des messages, appel cyclique
void SendUsbMessage(uint8_t *pData, uint16_t nbBytesToSend)
{
    uint16_t i = 0;
    uint16_t FreeSize = 0;
    //selon spec. CCITT il faut initialiser la valeur du Crc16 à  0xFFFF
   
    // Traitement émission à  introduire ICI
    // Formatage message et remplissage fifo émission
    FreeSize = GetWriteSpace(&descrUSBFifoTX);
    if (FreeSize >= nbBytesToSend)
    {
        for(i = 0; i < nbBytesToSend; i++)
        {
            // Dépose le message dans le fifo
            PutCharInFifo(&descrUSBFifoTX, *pData);
            if(*pData == 0x0A)
            {
                break;
            }
            pData++;
            
        }
        
//        PutCharInFifo(&descrUSBFifoTX, *pData);
//        do
//        {
//            pData++;
//            PutCharInFifo(&descrUSBFifoTX, *pData);
//        }while(*pData != 0x0A);
        
    }
    if (FreeSize > 0)
    {  
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    }
}

// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la réponse générée dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    uint8_t TXSize;
    uint8_t c;
    bool TxBuffFull;

    USART_ERROR UsartStatus;    

    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) )
    {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur à  la réception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) )
    {

        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0)
        {

            // Traitement RX à  faire ICI
            // Lecture des caractà¨res depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                c = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                PutCharInFifo(&descrGNSSFifoRX, c);
            }
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } 
        else 
        {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN)
            {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }
    } // end if RX
    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) 
    {
        // Traitement TX à  faire ICI
        TXSize = GetReadSize (&descrGNSSFifoTX);
        // Envoi des caractà¨res depuis le fifo SW -> buffer HW 
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratà¨res à  émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
        TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_1);
        if ((TXSize > 0) && TxBuffFull == false)
        {
            do 
            {
                GetCharFromFifo (&descrGNSSFifoTX, &c);
                PLIB_USART_TransmitterByteSend (USART_ID_1,c);
                TXSize = GetReadSize (&descrGNSSFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_1);
                
            } while ((TXSize > 0) && (TxBuffFull == false));
                
             
            // Clear the TX interrupt Flag (Seulement apres TX) 
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            if(TXSize == 0)
            {
                // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à  transmettre)
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            }
            
   
        }
        else
        {
            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        }
    }
 } // end_ISR Usart 1
 void __ISR(_UART_2_VECTOR, ipl4AUTO) _IntHandlerDrvUsartInstance1(void)
 {
    uint16_t TXSize;
    uint8_t c;
    bool TxBuffFull;

    USART_ERROR UsartStatus;    

    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_2_ERROR) )
    {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);
        // Traitement de l'erreur à  la réception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_2_RECEIVE) )
    {

        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_2);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0)
        {

            // Traitement RX à  faire ICI
            // Lecture des caractà¨res depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_2))
            {
                c = PLIB_USART_ReceiverByteReceive(USART_ID_2);
                PutCharInFifo(&descrUSBFifoRX, c);
            }
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
        } 
        else 
        {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN)
            {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_2);
            }
        }
    } // end if RX
    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT) ) 
    {
        // Traitement TX à  faire ICI
        TXSize = GetReadSize (&descrUSBFifoTX);
        // Envoi des caractà¨res depuis le fifo SW -> buffer HW 
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratà¨res à  émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
        TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_2);
        if ((TXSize > 0) && TxBuffFull == false)
        {
            do 
            {
                GetCharFromFifo (&descrUSBFifoTX, &c);
                PLIB_USART_TransmitterByteSend (USART_ID_2,c);
                TXSize = GetReadSize (&descrUSBFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull (USART_ID_2);
                
            } while ((TXSize > 0) && (TxBuffFull == false));
                
             
            // Clear the TX interrupt Flag (Seulement apres TX) 
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
            if(TXSize == 0)
            {
                // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à  transmettre)
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
            }
        }
        else
        {
            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
        }
    }
 }


