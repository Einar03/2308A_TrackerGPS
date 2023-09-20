/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_SDCard.h"
#include "Mc32gest_UART.h"
//#include "Mc32DriverLcd.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_SDCARD_DATA COHERENT_ALIGNED app_sdcardData;

SYS_FS_ERROR Error_value;
SYS_FS_ERROR Error_file;
SYS_FS_RESULT res;

static uint16_t Cnt = 0;

bool writeFlag = false;

uint8_t Datas[90];
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_SDCARD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_sdcardData.state = APP_MOUNT_DISK;
//    app_sdcardData.state = APP_IDLE;

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_SDCARD_Tasks ( void )
{
    // Variables locales
    uint8_t nbCharToWrite = 0;
    
    
    /* The application task state machine */
    switch(app_sdcardData.state)
    {
        case APP_MOUNT_DISK:
            if(SYS_FS_Mount("/dev/mmcblka1", "/mnt/myDrive", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * mounting again untill success. */

                Error_value = SYS_FS_Error();
                app_sdcardData.state = APP_MOUNT_DISK;
            }
            else
            {
                /* Mount was successful. Unmount the disk, for testing. */

                app_sdcardData.state = APP_UNMOUNT_DISK;
            }
            break;

        case APP_UNMOUNT_DISK:
            if(SYS_FS_Unmount("/mnt/myDrive") != 0)
            {
                /* The disk could not be un mounted. Try
                 * un mounting again untill success. */

                app_sdcardData.state = APP_UNMOUNT_DISK;
            }
            else
            {
                /* UnMount was successful. Mount the disk again */

                app_sdcardData.state = APP_MOUNT_DISK_AGAIN;
            }
            break;

        case APP_MOUNT_DISK_AGAIN:
            if(SYS_FS_Mount("/dev/mmcblka1", "/mnt/myDrive", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * mounting again untill success. */

                app_sdcardData.state = APP_MOUNT_DISK_AGAIN;
            }
            else
            {
                /* Mount was successful. Set current drive so that we do not have to use absolute path. */

                app_sdcardData.state = APP_SET_CURRENT_DRIVE;
            }
            break;

        case APP_SET_CURRENT_DRIVE:
            if(SYS_FS_CurrentDriveSet("/mnt/myDrive") == SYS_FS_RES_FAILURE)
            {
                /* Error while setting current drive */
                app_sdcardData.state = APP_ERROR;
            }
            else
            {
                /* Open a file for reading. */
                app_sdcardData.state = APP_OPEN_FIRST_FILE;
            }

        case APP_OPEN_FIRST_FILE:
//            app_sdcardData.fileHandle = SYS_FS_FileOpen("FILE_TOO_LONG_NAME_EXAMPLE_123.JPG",
//                    (SYS_FS_FILE_OPEN_READ));
            app_sdcardData.fileHandle = SYS_FS_FileOpen("test.txt",
                    (SYS_FS_FILE_OPEN_APPEND));
            if(app_sdcardData.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                Error_value = SYS_FS_Error();
                /* Could not open the file. Error out*/
                app_sdcardData.state = APP_ERROR;
            }
            else
            {
                /* Create a directory. */
                app_sdcardData.state = APP_CREATE_DIRECTORY;
            }
            break;

        case APP_CREATE_DIRECTORY:
//            if(SYS_FS_DirectoryMake("Dir1") == SYS_FS_RES_FAILURE)
//            {
//                /* Error while setting current drive */
//                app_sdcardData.state = APP_ERROR;
//            }
//            else
//            {
//                /* Open a second file for writing. */
//                app_sdcardData.state = APP_OPEN_SECOND_FILE;
//            }
            
//            res = SYS_FS_FilePrintf(app_sdcardData.fileHandle, "Hello World %d", Cnt);
            
            
//            GetGnssMessage(Datas, 20);
//            res = SYS_FS_FilePrintf(app_sdcardData.fileHandle,"%s", Datas);
            
            ClearBuffer();
            nbCharToWrite = GetGnssCmd(Datas);
            if(nbCharToWrite != 0)
            {
                res = SYS_FS_FilePrintf(app_sdcardData.fileHandle,"%s", Datas);
            }
            else
            {
                res = SYS_FS_RES_FAILURE;
            }
            
            Cnt++;
            
//            SYS_FS_FileWrite(app_sdcardData.fileHandle1, (const void *)app_sdcardData.data, 512) == -1;
            if(res == SYS_FS_RES_SUCCESS)
            {
//                if(Cnt == 10);
//                {
                app_sdcardData.state = APP_CLOSE_FILE;
//                }
            }
            else
            {
                Error_value = SYS_FS_Error();
                Error_file = SYS_FS_FileError(app_sdcardData.fileHandle);
                //app_sdcardData.state = APP_ERROR;
            }
                    
            break;

        case APP_OPEN_SECOND_FILE:
            /* Open a second file inside "Dir1" */
            app_sdcardData.fileHandle1 = SYS_FS_FileOpen("Dir1/FILE_TOO_LONG_NAME_EXAMPLE_123_1.JPG",
                    (SYS_FS_FILE_OPEN_WRITE));

            if(app_sdcardData.fileHandle1 == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                app_sdcardData.state = APP_ERROR;
            }
            else
            {
                /* Read from one file and write to another file */
                app_sdcardData.state = APP_READ_WRITE_TO_FILE;
            }

        case APP_READ_WRITE_TO_FILE:

            if(SYS_FS_FileRead(app_sdcardData.fileHandle, (void *)app_sdcardData.data, 512) == -1)
            {
                /* There was an error while reading the file.
                 * Close the file and error out. */

                SYS_FS_FileClose(app_sdcardData.fileHandle);
                app_sdcardData.state = APP_ERROR;
            }
            else
            {
                /* If read was success, try writing to the new file */
                if(SYS_FS_FileWrite(app_sdcardData.fileHandle1, (const void *)app_sdcardData.data, 512) == -1)
                {
                    /* Write was not successful. Close the file
                     * and error out.*/
                    SYS_FS_FileClose(app_sdcardData.fileHandle1);
                    app_sdcardData.state = APP_ERROR;
                }
                else if(SYS_FS_FileEOF(app_sdcardData.fileHandle) == 1)    /* Test for end of file */
                {
                    /* Continue the read and write process, untill the end of file is reached */

                    app_sdcardData.state = APP_CLOSE_FILE;
                }
            }
            break;

        case APP_CLOSE_FILE:
            /* Close both files */
            SYS_FS_FileClose(app_sdcardData.fileHandle);
//            SYS_FS_FileClose(app_sdcardData.fileHandle1);
             /* The test was successful. Lets idle. */
            if(Cnt < 65500)
            {
                app_sdcardData.state = APP_OPEN_FIRST_FILE;
                SDCard_RToggle();
            }
            else
            {
                app_sdcardData.state = APP_IDLE;
                SDCard_ROff();
            }
            break;

        case APP_IDLE:
            /* The appliction comes here when the demo
             * has completed successfully. Switch on
             * green LED. */
            //BSP_LEDOn(APP_SUCCESS_LED);
            break;
        case APP_ERROR:
            /* The appliction comes here when the demo
             * has failed. Switch on the red LED.*/
            //BSP_LEDOn(APP_FAILURE_LED);
            Error_value = SYS_FS_Error();
            break;
        default:
            break;

    }

} //End of APP_Tasks
APP_SDCARD_STATES SDCard_GetState(void)
{
    return app_sdcardData.state;
}
void APP_UpdateSDCardState(APP_SDCARD_STATES NewState)
{
    app_sdcardData.state = NewState;
}

void SetWriteFlag(void)
{
    writeFlag = true;
}

void ResetWriteFlag(void)
{
    writeFlag = false;
}

void ClearBuffer(void)
{
    uint8_t i;
    
    for(i=0; i < sizeof(Datas); i++)
    {
        Datas[i] = 0;
    }
}



/*******************************************************************************
 End of File
 */

