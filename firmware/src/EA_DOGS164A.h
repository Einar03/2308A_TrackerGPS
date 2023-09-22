/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */
#ifndef _EA_DOGS164A_H    /* Guard against multiple inclusion */
#define _EA_DOGS164A_H

#include "stdbool.h"
#include "stdint.h"
/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
    
#define BIT0                        0x01
#define BIT1                        0x02
#define BIT2                        0x04
#define BIT3                        0x08
#define BIT4                        0x10
#define BIT5                        0x20
#define BIT6                        0x40
#define BIT7                        0x80
    
// Defines pour le premier byte a envoyer au LCD
#define LCD_INST                    0xF8            // Envoie d'un instruction
#define LCD_BUSSY_FLAG              0xFA            // Lecture du flag busy
#define LCD_WRITE_DATA              0xFA            // Ecriture des données
#define LCD_READ_DATA               0xFE            // Lecture des données
#define TOP_VIEW                    0x05            // Orientation de l'écran top
#define BOTTOM_VIEW                 0x06            // Orientation de l'écran botttom

#define CHAR_TABLE_SELECT           0x72
#define DISPLAY_ON                  BIT2
#define DISPLAY_OFF                 0x00
#define SLEEP_ENABLE                0x02
#define SLEEP_DISABLE               0x00
#define BUS_4BITS                   false
#define BUS_8BITS                   true
 
#define DISPLAY_CONTROL             0x08
#define POWER_DISPLAY_CONTRAST      0x50
#define EXTENDED_FUNCTION_SET       0x08
#define FUNCTIONSET_RE0             0x20
#define FUNCTIONSET_RE1             0x22
    
#define FUNCTION_SET                0x20
#define CLEAR_LCD                   0x01
#define CURSOR_HOME                 0x20
#define PWRDOWN_MODE                0x20
#define ENTRY_MODE                  0x40


#define CURSOR_DISPLAY_SHIFT        0x10

    


    
    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */
    
    typedef enum
    {
        INSTRUCTION      = 0x00, // RS = 0, RW = 0
        READ_BUSY_FLAG   = 0x01, // RS = 0, RW = 1
        DATA_WRITE       = 0x02, // RS = 1, RW = 0
        DATA_READ        = 0x03, // RS = 1, RW = 1
    }RS_RW;
    
    typedef struct _example_struct_t {
        /* Describe structure member. */
        int some_number;

        /* Describe structure member. */
        bool some_flag;

    } example_struct_t;
    
    typedef struct
    {   
        uint8_t startByte;
        uint8_t convertedData_Lsb;
        uint8_t convertedData_Msb;
    }s_EADOG_Data;
    
    typedef union
    {
        uint8_t data;
        struct
        {
            uint8_t dataLsb : 4;
            uint8_t dataMsb : 4;
        }data4bits;
    }U_valData;
    
    typedef struct
    {
        uint8_t displayControl;
        uint8_t power_Display_Contrast;
        uint8_t extendedFunctionSet;
        uint8_t functionSet_RE0;
        uint8_t functionSet_RE1;
    }s_EADOGS_REG_VAL;
    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

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
    int ExampleFunction(int param1, int param2);
    
    void LCD_EADOGS_Init(s_EADOGS_REG_VAL *registerValue, bool dataLength, uint8_t nbLines, uint8_t viewOrientation, bool fontWidth, bool invertCursor);
    void LCD_EADOGS_ChangeView(s_EADOGS_REG_VAL *registerValue, uint8_t viewOrientation);
    void LCD_EADOGS_ContrastSet(s_EADOGS_REG_VAL *registerValue, uint8_t contrast);
    void LCD_EADOGS_CharTableSelect(s_EADOGS_REG_VAL *registerValue, uint8_t table);
    void LCD_EADOGS_LinesSet(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines);
    void LCD_EADOGS_Clear(void);
    void LCD_EADOGS_Sleep(s_EADOGS_REG_VAL *registerValue);
    void LCD_EADOGS_WakeUp(s_EADOGS_REG_VAL *registerValue);
    void LCD_CursorHome(s_EADOGS_REG_VAL *registerValue);
    void LCD_EADOGS_GoTo(s_EADOGS_REG_VAL *registerValue, uint8_t x, uint8_t y);
    void LCD_EADOGS_GoHome(s_EADOGS_REG_VAL *registerValue);
    void LCD_Printf(const char *format, ...);
    
    void LCD_EADOGS_SendData(RS_RW operation, uint8_t functionVal);
    
    void SetNbLines(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines);
    void SetIS(s_EADOGS_REG_VAL *registerValue);
    void ResetIS(s_EADOGS_REG_VAL *registerValue);
    void ConvertDataToSend(U_valData dataToConvert, s_EADOG_Data *convertedData);
    
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
