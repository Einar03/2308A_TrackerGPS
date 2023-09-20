/* ************************************************************************** */
/** gestion SPI via machine d'�tat
 *
 *  @Company
 *    ETML-ES - SCA
 *
 *  @File Name
 *    Mc32Ex8_2_spi_sm.c
 *
 *  @Summary
 *    gestion SPI via machine d'�tat
 *
 *  @Description
 *    gestion SPI via machine d'�tat
 *    Test� pour fonctionner avec LM70 sur SPI1
 * 
 *   Cr�ation 14.03.2017 SCA
 *  
*/

#include "Mc32_SPI_StateMachine.h"
//#include "bsp.h"
#include "peripheral/spi/plib_spi.h"
#include "system/clk/sys_clk.h" // pour SYS_CLK_PeripheralFrequencyGet()
#include "system/int/sys_int.h"
//#include "system_config.h"
//#include "system_config/default/system_config.h"
#include "system_config/default/system_definitions.h"


//byte bidon pour envoi lorsqu'uniquement une lecture est demand�e
//le spi �tant full-duplex, une lecture implique forc�ment une �criture simultan�e
#define DUMMY_BYTE  0x81   

// SPI_ID_1 correspond au SPI 1 !
#define KitSpi1 (SPI_ID_1)

SPI_STATES spiState = SPI_STATE_UNINITIALIZED;

//fonction � appeler 1x au d�marrage pour init.
//code repris de la g�n�ration du driver par Hamony 1.08
void SPI_Init(void)
{
    // *** init du p�riph. SPI ***
    PLIB_SPI_Disable(KitSpi1);

    PLIB_SPI_BufferClear(KitSpi1);
    PLIB_SPI_StopInIdleDisable(KitSpi1);
    PLIB_SPI_PinEnable(KitSpi1, SPI_PIN_DATA_OUT);
    PLIB_SPI_CommunicationWidthSelect(KitSpi1, SPI_COMMUNICATION_WIDTH_8BITS);
    // LM70 MAX 6.25 MHz choix 5 MHz
    PLIB_SPI_BaudRateSet(KitSpi1, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1), 1000000);
    // Config polarit� traitement des signaux SPI
    // pour input � confirmer
    // Polarit� clock OK
    // Phase output � confirmer
    PLIB_SPI_InputSamplePhaseSelect(KitSpi1, SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE );
    PLIB_SPI_ClockPolaritySelect(KitSpi1, SPI_CLOCK_POLARITY_IDLE_HIGH);
    PLIB_SPI_OutputDataPhaseSelect(KitSpi1, SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK);
    PLIB_SPI_MasterEnable(KitSpi1);
    PLIB_SPI_FramedCommunicationDisable(KitSpi1);
    PLIB_SPI_FIFOEnable(KitSpi1);     // Enhenced buffer mode

    PLIB_SPI_Enable(KitSpi1);

   
   // action de configuration
//    if (SPI_GetState() == SPI_STATE_IDLE)               
//    {
//        SPI_StartRead(4);               
//    }
//    else if (SPI_GetState() == SPI_STATE_IDLE_READ_DATA_AVAILABLE)               
//    {
//        SPI_ReadByte();
//        SPI_ReadByte();
//        SPI_ReadByte();
//        SPI_ReadByte();
//    }
 
}

//Ecriture.
//Comme le SPI est obligatoirement full-duplex,
//les donn�es re�ues ne seront pas trait�es
void SPI_StartWrite(uint32_t nBytes, uint8_t* pBytesToWrite)
{
    // Variables locales
    uint8_t i;
    
    CS_LCDOff();
       // Ecriture des bytes
    for(i=0; i<nBytes; i++)
    {
        PLIB_SPI_BufferWrite(KitSpi1, *pBytesToWrite);
        pBytesToWrite++;
    }
    spiState = SPI_STATE_BUSY_WRITE;
}

//Lecture/�criture.
//Comme le SPI est obligatoirement full-duplex,
//des donn�es sont re�ues simultan�ment � l'envoi
//void SPI_StartReadWrite(uint32_t nBytes, uint8_t* pBytesToWrite)
//{
//     // Variables locales
//    uint8_t i;
//    
//    CS_LCDOff();
//    // Ecriture des bytes
//    for(i=0; i<nBytes; i++)
//    {
//        PLIB_SPI_BufferWrite(KitSpi1, *pBytesToWrite);
//        pBytesToWrite++;
//    }
//    spiState = SPI_STATE_BUSY_READ_WRITE;
//}

//Lecture.
//Comme le SPI est obligatoirement full-duplex,
//il faut envoyer des donn�es bidons pour faire une lecture
void SPI_StartRead(uint32_t nBytes)
{
    // Variables locales
    uint8_t i;
    
    CS_LCDOff();
    
    // Ecriture des bytes
    for(i=0; i<nBytes; i++)
    {
        PLIB_SPI_BufferWrite(KitSpi1, DUMMY_BYTE);
    }
    spiState = SPI_STATE_BUSY_READ;    
}

//pour obtenir l'�tat interne de la SM spi
SPI_STATES SPI_GetState (void)
{
    return spiState;
}

//lecture d'un byte dans buffer r�ception
uint8_t SPI_ReadByte(void)
{
    uint8_t data;
    
    data =  PLIB_SPI_BufferRead(KitSpi1);
    if (PLIB_SPI_ReceiverFIFOIsEmpty(KitSpi1))
    {
        spiState = SPI_STATE_IDLE;   
    }
    return (data);
}

//fonction � appeler p�riodiquement pour gestion SPI
//gestion de la machine d'�tat du SPI
void SPI_DoTasks(void)
{
    switch(spiState)
    {
        case SPI_STATE_UNINITIALIZED:
            SPI_Init();
            spiState = SPI_STATE_IDLE;
            break;
        case SPI_STATE_IDLE:
            // Rien faire
            break;
        case SPI_STATE_IDLE_READ_DATA_AVAILABLE:
            // Rien faire
            break;
        case SPI_STATE_BUSY_WRITE:
            if(!PLIB_SPI_IsBusy(KitSpi1))
            {
               CS_LCDOn();
               spiState = SPI_STATE_IDLE;
            }
            break;
        case SPI_STATE_BUSY_READ_WRITE:
            if(!PLIB_SPI_IsBusy(KitSpi1) && !PLIB_SPI_ReceiverFIFOIsEmpty(KitSpi1))
            {
                CS_LCDOn();
                spiState = SPI_STATE_IDLE_READ_DATA_AVAILABLE;
            }
            break;
        case SPI_STATE_BUSY_READ:
            if(!PLIB_SPI_IsBusy(KitSpi1) && !PLIB_SPI_ReceiverFIFOIsEmpty(KitSpi1))
            {
                CS_LCDOn();
                spiState = SPI_STATE_IDLE_READ_DATA_AVAILABLE;
            }
            break;
        default:
            break;
    }
}


/* *****************************************************************************
 End of File
 */
