#include "dacxx65.h"

static void DACXX65_Write24b(DACXX65_t *, uint32_t);

void DACXX65_Init(DACXX65_t *dac, DACXX65_BIT_t bit, void (*fp_spi_tx)(uint32_t), void (*fp_sync_enable)(bool))
{
    dac->fp_sync_enable = fp_sync_enable;
    dac->fp_spi_tx = fp_spi_tx;
    dac->datawidth = bit;
}

void DACXX65_SetReference(DACXX65_t *dac, uint16_t reference)
{
    uint32_t data = DACXX65_MASK_PD0;

    data |= reference;

    DACXX65_Write24b(dac, data);
}

void DACXX65_SetChannelPower(DACXX65_t *dac, DACXX65_CHANNEL_t ch, uint16_t power)
{
    uint32_t data = power | DACXX65_MASK_PD0;

    if (ch == DACXX65_CHANNEL_A)
        data |= DACXX65_SINGLE_CHANNEL_UPDATE;

    else if (ch == DACXX65_CHANNEL_B)
        data |= DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL0;

    else if (ch == DACXX65_CHANNEL_C)
        data |= DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL1;

    else if (ch == DACXX65_CHANNEL_D)
        data |= DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL1 | DACXX65_MASK_DACSEL0;

    else if (ch == DACXX65_CHANNEL_ALL)
        data |= DACXX65_BROADCAST_UPDATE | DACXX65_MASK_DACSEL1;

    DACXX65_Write24b(dac, data);
}

void DACXX65_WriteChannel(DACXX65_t *dac, DACXX65_CHANNEL_t ch, uint16_t value)
{
    uint32_t data = 0;

    switch (ch)
    {
    case DACXX65_CHANNEL_A:
        data = DACXX65_SINGLE_CHANNEL_UPDATE;
        break;
    case DACXX65_CHANNEL_B:
        data = DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL0;
        break;
    case DACXX65_CHANNEL_C:
        data = DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL1;
        break;
    case DACXX65_CHANNEL_D:
        data = DACXX65_SINGLE_CHANNEL_UPDATE | DACXX65_MASK_DACSEL1 | DACXX65_MASK_DACSEL0;
        break;
    case DACXX65_CHANNEL_ALL:
        data = DACXX65_BROADCAST_UPDATE | DACXX65_MASK_DACSEL1;
        break;
    default:
        return;
    }

    switch (dac->datawidth)
    {
    case DACXX65_12BIT:
        data |= (value << 4) & (0xFFFF);
        break;
    case DACXX65_14BIT:
        data |= (value << 2) & (0xFFFF);
        break;
    case DACXX65_16BIT:
        data |= (value << 0) & (0xFFFF);
        break;
    default:
        return;
        break;
    }

    DACXX65_Write24b(dac, data);
}

static void DACXX65_Write24b(DACXX65_t *dac, uint32_t data)
{
    // Write Right aligned 24bit data

    void (*const SYNC)(bool) = dac->fp_sync_enable;

    if (SYNC != NULL)
    	SYNC(true);

    dac->fp_spi_tx(data);

    if (SYNC != NULL)
    	SYNC(false);
}
