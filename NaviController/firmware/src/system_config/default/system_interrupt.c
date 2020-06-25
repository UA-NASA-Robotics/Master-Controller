/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"
#include "changeNotification.h"
#include "Timers.h"
#include "../../CAN_Handler/CAN.h"
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
 
void __ISR(_UART2_TX_VECTOR, ipl5AUTO) _IntHandlerDrvUsartTransmitInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
}
void __ISR(_UART2_RX_VECTOR, ipl1AUTO) _IntHandlerDrvUsartReceiveInstance0(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart0);
}
void __ISR(_UART2_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance0(void)
{
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 

 
void __ISR(_UART4_TX_VECTOR, ipl5AUTO) _IntHandlerDrvUsartTransmitInstance1(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart1);
}
void __ISR(_UART4_RX_VECTOR, ipl5AUTO) _IntHandlerDrvUsartReceiveInstance1(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart1);
}
void __ISR(_UART4_FAULT_VECTOR, ipl5AUTO) _IntHandlerDrvUsartErrorInstance1(void)
{
    DRV_USART_TasksError(sysObj.drvUsart1);
}
 
 

 
void __ISR(_UART3_TX_VECTOR, ipl1AUTO) _IntHandlerDrvUsartTransmitInstance2(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart2);
}
void __ISR(_UART3_RX_VECTOR, ipl4AUTO) _IntHandlerDrvUsartReceiveInstance2(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart2);
}
void __ISR(_UART3_FAULT_VECTOR, ipl4AUTO) _IntHandlerDrvUsartErrorInstance2(void)
{
    DRV_USART_TasksError(sysObj.drvUsart2);
}

 

 

 

 
 void __ISR(_CHANGE_NOTICE_B_VECTOR, ipl1AUTO) _IntHandlerChangeNotification_PortB(void)
{
    if(getLoadedState())
    {
       //pinChangeNotified();
    }
    PORTB;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_B);
}
   void __ISR(_CHANGE_NOTICE_C_VECTOR, ipl1AUTO) _IntHandlerChangeNotification_PortC(void)
{
    if(getLoadedState())
    {
      // pinChangeNotified();
    }
    PORTC;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_C);
}
   void __ISR(_CHANGE_NOTICE_D_VECTOR, ipl1AUTO) _IntHandlerChangeNotification_PortD(void)
{
    if(getLoadedState())
    {
       //pinChangeNotified();
    }
    PORTD;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_D);
}

 

void __ISR(_TIMER_1_VECTOR, ipl6AUTO) IntHandlerDrvTmrInstance0(void)
{
    globalTimerTracker( );
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_1);
}
 
void __ISR(_CAN1_VECTOR, IPL6AUTO) _IntHandlerDrvCANInstance0(void)
{
    CAN_ISR_CALLBACK();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_CAN_1);
}



/*******************************************************************************
 End of File
*/
