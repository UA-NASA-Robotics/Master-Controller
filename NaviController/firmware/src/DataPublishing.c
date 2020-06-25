#include <stddef.h>
#include "DataPublishing.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "Definitions.h"
#include "Timers.h"

#define isINITbit   0
#define DATA_ELEMENTS_COUNT 7
timers_t dataPeriodTimer[DATA_ELEMENTS_COUNT];
int (*dataRetrievalFunc[DATA_ELEMENTS_COUNT])();
/* Clears all the get funciton pointers and transmission timers */
void InitDataPublishing()
{
    int i;
    for( i = 0;i< DATA_ELEMENTS_COUNT;i++)
    {
        dataRetrievalFunc[i] = NULL;
        dataPeriodTimer[i].timerInterval = 0;
    }
}
/** \brief: Adds the function pointer to get data and the interval to
 *          send the data over the can bus
 *
 * \        _index:  placement of 1 of 7 indexes allowed for each device in the global can bus
 * \getFuncPointer:  Function pointer to a getter function that returns the data you want to send 
 * \     _interval:  The frequency that you want to send the data
 * 
 * \return: N\A
 */
void initGlobalData(GlobalDeviceData_t _index, int (*getFuncPointer)(void), unsigned long _interval) {
    setTimerInterval(&dataPeriodTimer[_index], _interval);
    dataRetrievalFunc[_index] = getFuncPointer;

}
/** \brief: publish the data that has been set up using the 'initGlobalData'
 *
 * \return: return true if data has been sent
 */
bool publishData() {
    int i;
    bool validData = false;
    // Loop through all the elements that we could potentially send
    for (i = 0; i < DATA_ELEMENTS_COUNT; i++) {
        // if an data element hasn't been initialized then the pointer to the data retrieval function will be NULL
        if (dataRetrievalFunc[i] != NULL) {

            // Make sure the interval that we want to send the data out on has elapsed
            if (timerDone(&(dataPeriodTimer[i]))) {
                // Making sure we actually have data to send
                validData = true;
                //Send the data on the can bus
                ToSendCAN(i + DATA_ELEMENTS_COUNT*MASTER_CONTROLLER, (dataRetrievalFunc[i])());
            }
        }
    }
    if (validData) {
        sendDataCAN(GLOBAL_ADDRESS);
    }
    return validData;
}

