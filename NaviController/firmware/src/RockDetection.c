#include "RockDetection.h"
#include "FastTransfer.h"

/* [06][85][sender][receiver][msg length] [4] [lower byte] [upper byte] .... [CRC]*/

#define MAX_NUM_OBJSCT 5
void getRockLocations()
{
    int rockCount = PozyxFT.ReceivedData[4];
    
    int i;
    for(i = 5;i < 5+rockCount*2;i+=2)
    {
        addRock(PozyxFT.ReceivedData[i],PozyxFT.ReceivedData[i+1]);
    }
    
}





