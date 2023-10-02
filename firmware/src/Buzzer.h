/* ************************************************************************** */
/** Descriptive File Name

  Auteur : Einar Farinas
  Date : 25/09/2023

  @File Name
    EA_DOGS164A.h

  @Summary
    Librarie pour commander  avec le BUS SPI des écrans EA DOGS utilisant
	le controlleur SSD1803A
 */
/* ************************************************************************** */
#ifndef _EA_DOGS164A_H    /* Guard against multiple inclusion */
#define _EA_DOGS164A_H

#include "stdbool.h"     // Pour les types bool
#include "stdint.h"      // Pour les types definis des variables int8_t, uint8_t, ... 
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

// Pour masquage des bits
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

// Pour la sélection de la table de caracteres du LCD
// Cyrillic, anglais-japonais et européen
#define CHAR_TABLE_SELECT           0x72 
// Bits pour controller le LCD          
#define DISPLAY_ON                  BIT2
#define DISPLAY_OFF                 0x00
#define SLEEP_ENABLE                0x02
#define SLEEP_DISABLE               0x00
#define BUS_4BITS                   false
#define BUS_8BITS                   true
 
// Valeurs des registres du LCd par défaut
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
    
	// Enumération pour les valeurs du start byte
    typedef enum
    {
        INSTRUCTION      = 0x00, // RS = 0, RW = 0
        READ_BUSY_FLAG   = 0x01, // RS = 0, RW = 1
        DATA_WRITE       = 0x02, // RS = 1, RW = 0
        DATA_READ        = 0x03, // RS = 1, RW = 1
    }RS_RW;
    
    // Structure pour la trame a envoyer
	// avec separation du byte de donnée
    typedef struct
    {   
        uint8_t startByte;
        uint8_t convertedData_Lsb;
        uint8_t convertedData_Msb;
    }s_EADOG_Data;
    
	// Union pour diviser un byte en deux bytes
	// pour l'envoi de données au LCD
    typedef union
    {
        uint8_t data;
        struct
        {
            uint8_t dataLsb : 4;
            uint8_t dataMsb : 4;
        }data4bits;
    }U_valData;
    
	// Structure pour stocker la valeur de chaque registré utilisé
	// Ceci a été implémetée pour faire du masquage sur les bits requis
	// Sans changer les autres registres
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
   
    
	// Fonction pour l'initalisation du LCD
	// avec paramétres de configuration
    void LCD_EADOGS_Init(s_EADOGS_REG_VAL *registerValue, bool dataLength, uint8_t nbLines, uint8_t viewOrientation, bool fontWidth, bool invertCursor);
	// Fonction pour changer l'orientation de l'écran LCD
    void LCD_EADOGS_ChangeView(s_EADOGS_REG_VAL *registerValue, uint8_t viewOrientation);
	// Fonction pour changer le contraste du LCD
    void LCD_EADOGS_ContrastSet(s_EADOGS_REG_VAL *registerValue, uint8_t contrast);
	// Fonction pour changer la table de caractères
    void LCD_EADOGS_CharTableSelect(s_EADOGS_REG_VAL *registerValue, uint8_t table);
	// Fonction pour changer le nombre de linges sur le LCD
    void LCD_EADOGS_LinesSet(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines);
	// Fonction pour effacer l'écran LCD
    void LCD_EADOGS_Clear(void);
	// Fonction pour mettre l'écran LCD en état de veille
    void LCD_EADOGS_Sleep(s_EADOGS_REG_VAL *registerValue);
	// Fonction pour reveiller l'écran LCD
    void LCD_EADOGS_WakeUp(s_EADOGS_REG_VAL *registerValue);
	// Fonction pour déplacer le curseur à la position initial
    void LCD_EADOGS_GoHome(s_EADOGS_REG_VAL *registerValue);
	// Fonction pour déplacer le curseur à la position voulue
    void LCD_EADOGS_GoTo(s_EADOGS_REG_VAL *registerValue, uint8_t x, uint8_t y);
	// Fonction pour écrire sur l'écran LCD
    void LCD_Printf(const char *format, ...);
    
	// Fonction pour envoyer la trame au SPI
    void LCD_EADOGS_SendData(RS_RW operation, uint8_t functionVal);
    // Fonction pour changer la valeur des registres en fonction du nombre de lignes 
	// voulus
    void SetNbLines(s_EADOGS_REG_VAL *registerValue, uint8_t nbLines);
	// Fonction pour set le registre IS
    void SetIS(s_EADOGS_REG_VAL *registerValue);
	// Fonction reset le registre IS
    void ResetIS(s_EADOGS_REG_VAL *registerValue);
	// Fonction pour inverser les bits du byte de donnée afin de les evoyer au LCD
    void ConvertDataToSend(U_valData dataToConvert, s_EADOG_Data *convertedData);
	// Fonction pour activer le clignotement du curseur
    void LCD_BlinkCursor(s_EADOGS_REG_VAL *registerValue, bool enable);
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
