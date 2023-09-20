/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "EA_DOGS164A.h"
#include "Mc32Delays.h"
//#include "system_config/default/system_config.h"
#include "Mc32_SPI_StateMachine.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "system_config/default/system_definitions.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

// Tableau pour inverser les bytes MSB-LSB => LSB-MSB
static const uint8_t BitReverseTable256[256] = 
{
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

//s_EADOG_Data dataToSend;
//U_valData data;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */

/** 
  @Function
    int ExampleLocalFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Description
    Full description, explaining the purpose and usage of the function.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

  @Precondition
    List and describe any required preconditions. If there are no preconditions,
    enter "None."

  @Parameters
    @param param1 Describe the first parameter to the function.
    
    @param param2 Describe the second parameter to the function.

  @Returns
    List (if feasible) and describe the return values of the function.
    <ul>
      <li>1   Indicates an error occurred
      <li>0   Indicates an error did not occur
    </ul>

  @Remarks
    Describe any special behavior not described above.
    <p>
    Any additional remarks.

  @Example
    @code
    if(ExampleFunctionName(1, 2) == 0)
    {
        return 3;
    }
 */
void LCD_EADOGS_Init(s_EADOGS_REG_VAL *registerValue, bool dataLength, uint8_t nbLines, uint8_t viewOrientation, bool fontWidth, bool invertCursor)
{
    // Attente de >50ms apr�s allumage du syst�me
    LCD_ResetOn();
    delay_msCt(50);
    // Reset du LCD
    LCD_ResetOff();
    delay_msCt(20);
    LCD_ResetOn();
    delay_msCt(5);
    
    
    // 4 ou 8 bits de donn�e
    if(dataLength)
    {
        registerValue->functionSet_RE0 |= BIT4;
        registerValue->functionSet_RE1 |= BIT4;
    }
//    else
//    {
//        registerValue->functionSet_RE0 &= ~BIT4;
//        registerValue->functionSet_RE1 &= ~BIT4;
//    }
    // chagemenent des registres pour le nombre de lignes
    SetNbLines(registerValue, nbLines);
    if(fontWidth)
    {
        registerValue->extendedFunctionSet |= BIT2;
    }
//    else
//    {
//        registerValue->extendedFunctionSet &= ~BIT2;
//    }
    if(invertCursor)
    {
        registerValue->extendedFunctionSet |= BIT1;
    }
//    else
//    {
//        registerValue->extendedFunctionSet &= ~BIT1;
//    }
    
    // Fonction Set
    // 8 bit data et choix de lignes en fonction de nBLines ( 0 � 3 bit LSB)
//    registerValue->functionSet_RE1 = 0x3A;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    delay_msCt(1);
    // Extended function set 
    // choix de lines (0 � 3 bit MSB)
//    registerValue->extendedFunctionSet = 0x09;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->extendedFunctionSet); // 4 lignes
    delay_msCt(1);
    // Clear LCD
    LCD_EADOGS_Clear();
    delay_msCt(1);
    // Entry mode set
    // Sens d'affichage, top ou bottom
//    viewOrientation = 0x06;
    LCD_EADOGS_SendData(INSTRUCTION, viewOrientation);
    delay_msCt(1);
    // Bias setting
    LCD_EADOGS_SendData(INSTRUCTION, 0x1E); // BS1 = 1
    delay_msCt(1);
    
    SetIS(registerValue);
    // Function Set
//    registerValue->functionSet_RE0 = 0x39;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    delay_msCt(1);
    ResetIS(registerValue);
    // Inernat OSC
    LCD_EADOGS_SendData(INSTRUCTION, 0x1B); // BS0 = 1, 1/6 Bias
    delay_msCt(1);
    // Follower control
    LCD_EADOGS_SendData(INSTRUCTION, 0x6C);
    delay_msCt(1);
    // Power control
    registerValue->power_Display_Contrast = 0x54;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->power_Display_Contrast);
    delay_msCt(1);
    // Contrast set
    LCD_EADOGS_SendData(INSTRUCTION, 0x71);
    delay_msCt(1);
    // Function set
//    registerValue->functionSet_RE0 = 0x38;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    delay_msCt(1);
    // Display on
    registerValue->displayControl = 0x0F;
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->displayControl);
    delay_msCt(1);
    
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
void LCD_EADOGS_ChangeView(s_EADOGS_REG_VAL *registerValue, uint8_t viewOrientation)
{
    // Instruction set
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    // Changement de view
    LCD_EADOGS_SendData(INSTRUCTION, viewOrientation);
    // Instruction set
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
}


void LCD_EADOGS_ContrastSet(s_EADOGS_REG_VAL *registerValue, uint8_t contrast)
{
    if(contrast > 0x3F)
    {
        contrast = 0x3F;
    }
    
    SetIS(registerValue);
    registerValue->power_Display_Contrast |= (contrast >> 4);
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->power_Display_Contrast);
    LCD_EADOGS_SendData(INSTRUCTION, (0x7A|(contrast &= 0x0F)));
    ResetIS(registerValue);
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
}

void LCD_EADOGS_CharTableSelect(s_EADOGS_REG_VAL *registerValue, uint8_t table)
{
    if(table > 2)
    {
        table = 2;
    }
    
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    LCD_EADOGS_SendData(INSTRUCTION, CHAR_TABLE_SELECT); // 0x72
    LCD_EADOGS_SendData(DATA_WRITE, (table << 2));
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
}
void LCD_EADOGS_LinesSet(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines)
{
    SetNbLines(registerValue, nbLines);
    // 8 bit data et choix de lignes en fonction de nBLines ( 0 � 3 bit LSB)
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    // Extended function set 
    // choix de lines (0 � 3 bit MSB)
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->extendedFunctionSet); // 4 lignes
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
}
void LCD_EADOGS_Clear(void)
{
    LCD_EADOGS_SendData(INSTRUCTION, CLEAR_LCD);
}
void LCD_EADOGS_Sleep(s_EADOGS_REG_VAL *registerValue)
{
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, (registerValue->displayControl & ~BIT2));
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    LCD_EADOGS_SendData(INSTRUCTION, SLEEP_ENABLE);  // Bit1 indiqu� dans le datasheet � v�rifier si fonction
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0); // optional peut �tre
}
void LCD_EADOGS_WakeUp(s_EADOGS_REG_VAL *registerValue)
{
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE1);
    LCD_EADOGS_SendData(INSTRUCTION, SLEEP_DISABLE); // Bit1 indiqu� dans le datasheet � v�rifier si fonction
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, (registerValue->displayControl | BIT2));
}
void LCD_CursorHome(s_EADOGS_REG_VAL *registerValue)
{
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, 0x02);
}
void LCD_EADOGS_SendData(RS_RW operation, uint8_t functionVal)
{
    s_EADOG_Data dataToSend;
    
    // S�lection du byte de start
    switch(operation)
    {
        case INSTRUCTION :
            dataToSend.startByte = LCD_INST;
            break;
        case READ_BUSY_FLAG :
            dataToSend.startByte = LCD_BUSSY_FLAG;
            break;
        case DATA_WRITE :
            dataToSend.startByte = LCD_WRITE_DATA;
            break;
        case DATA_READ :
            dataToSend.startByte = LCD_READ_DATA;
            break;
        default:
            break;
    }
      
    // Byte de start
    // Conversion MSB-LSB => LSB-MSB pour envoyer les valeurs sur le SPI
    // plus s�paration du MSB et LSB en deux bytes
    ConvertDataToSend((U_valData)functionVal, &dataToSend);
    // envoi de la commande
    SPI_StartWrite(3, &dataToSend.startByte);
}

void LCD_BlinkCursor(s_EADOGS_REG_VAL *registerValue, bool enable)
{
    if(enable)
    {
        registerValue->displayControl |= BIT0;
    }
    else
    {
        registerValue->displayControl &= ~BIT0;
    }
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->displayControl);
}

void LCD_EADOGS_GoTo(s_EADOGS_REG_VAL *registerValue, uint8_t x, uint8_t y)
{
    LCD_EADOGS_SendData(INSTRUCTION, registerValue->functionSet_RE0);
    LCD_EADOGS_SendData(INSTRUCTION, (0x80 | (x+16*(y-1))));
}

void LCD_Printf(const char *format, ...)
{
    uint8_t i = 0;
    
    char Buffer[100];
    va_list args;
    va_start(args, format);

    vsprintf(Buffer, format, args);
    while(Buffer[i] != 0)
    {
        LCD_EADOGS_SendData(DATA_WRITE, Buffer[i]);
        i++;
        delay_msCt(5);
    }
    va_end(args);
    for(i=0; i<100; i++)
    {
        Buffer[i] = 0;
    }
}
void SetNbLines(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines)
{
    switch(nbLines)
    {
        // 1 ligne
        case 1:
            registerValue->functionSet_RE0 &=  ~BIT3;      // LBB
            registerValue->functionSet_RE1 &=  ~BIT3;      // LBB
            registerValue->extendedFunctionSet &=  ~BIT0;  // MSB
            break;
        // 2 lignes
        case 2:
            registerValue->functionSet_RE0 |=  BIT3;       // LSB
            registerValue->functionSet_RE1 &=  ~BIT3;      // LBB
            registerValue->extendedFunctionSet &=  ~BIT0;  // MSB
            break;
        // 3 lignes
        case 3:
            registerValue->functionSet_RE0 &=  ~BIT3;      // LSB
            registerValue->functionSet_RE1 &=  ~BIT3;      // LBB
            registerValue->extendedFunctionSet |=  BIT0;  // MSB
            break;
        // 4 lignes
        case 4:
            registerValue->functionSet_RE0 |=  BIT3;      // LSB
            registerValue->functionSet_RE1 |=  BIT3;      // LBB
            registerValue->extendedFunctionSet |=  BIT0;  // MSB
            break;
        default:
            break;
    }
}


//void SetRE(s_EADOGS_REG_VAL *registerValue)
//{
//    registerValue->functionSet_RE0 |= BIT1;
//}
//void ResetRE(s_EADOGS_REG_VAL *registerValue)
//{
//    registerValue->functionSet_RE0 &= ~BIT1;
//}
void SetIS(s_EADOGS_REG_VAL *registerValue)
{
    registerValue->functionSet_RE0 |= BIT0;
}
void ResetIS(s_EADOGS_REG_VAL *registerValue)
{
    registerValue->functionSet_RE0 &= ~BIT0;
}



void ConvertDataToSend(U_valData dataToConvert, s_EADOG_Data *convertedData)
{
    // Inversion des bits MSB-LSB -> LSB-MSB
    dataToConvert.data = BitReverseTable256[dataToConvert.data];
    // D�calage des bits pour avoir le bon format d'envoie sur le SPI
    convertedData->convertedData_Lsb = dataToConvert.data4bits.dataMsb << 4;
    convertedData->convertedData_Msb = dataToConvert.data4bits.dataLsb << 4;
}
/* *****************************************************************************
 End of File
 */
