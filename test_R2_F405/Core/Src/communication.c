#include "communication.h"
#include "can.h"
#include "can_bsp.h"
#include "stdio.h"

int16_t centerInfo[4];


void centerDecode(uint32_t StdId, uint8_t *rxdata)
{

    if (StdId == 0x050)
    {
        centerInfo[0] = *(uint16_t *)(rxdata) >> 4;
        centerInfo[1] = *(uint16_t *)(rxdata + 2);
        centerInfo[2] = *(uint16_t *)(rxdata + 4);
        centerInfo[3] = *(uint16_t *)(rxdata + 6);
    }
}
