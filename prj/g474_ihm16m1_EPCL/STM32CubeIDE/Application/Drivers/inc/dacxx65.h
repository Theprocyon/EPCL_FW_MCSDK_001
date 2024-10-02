
#ifndef _DACXX65_H_
#define _DACXX65_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DACXX65_REFERENCE_ALWAYS_POWERED_DOWN 0x2000
#define DACXX65_REFERENCE_POWERED_TO_DEFAULT 0x0000
#define DACXX65_REFERENCE_ALWAYS_POWERED_UP 0x1000

#define DACXX65_DATA_INPUT_REGISTER 0x011000

#define DACXX65_MASK_LD1 0x200000
#define DACXX65_MASK_LD0 0x100000
#define DACXX65_MASK_DACSEL1 0x040000
#define DACXX65_MASK_DACSEL0 0x020000
#define DACXX65_MASK_PD0 0x010000
#define DACXX65_MASK_PD1 0x008000
#define DACXX65_MASK_PD2 0x004000
#define DACXX65_MASK_DATA 0x00FFF0

#define DACXX65_SINGLE_CHANNEL_STORE    0                               /* LD1=0,LD0=0 */
#define DACXX65_SINGLE_CHANNEL_UPDATE   DACXX65_MASK_LD0               /* LD1=0,LD0=1 */
#define DACXX65_SIMULTANEOUS_UPDATE     DACXX65_MASK_LD1                 /* LD1=1,LD0=0 */
#define DACXX65_BROADCAST_UPDATE        DACXX65_MASK_LD1 | DACXX65_MASK_LD0 /* LD1=1,LD0=1 */

typedef enum
{
    DACXX65_CHANNEL_A = 0,
    DACXX65_CHANNEL_B,
    DACXX65_CHANNEL_C,
    DACXX65_CHANNEL_D,
    DACXX65_CHANNEL_ALL
} DACXX65_CHANNEL_t;

typedef enum
{
    DACXX65_16BIT = 0, // DAC8565
    DACXX65_14BIT,     // DAC8165
    DACXX65_12BIT,     // DAC7565
} DACXX65_BIT_t;

typedef struct __DACXX65_T_
{
    void (*fp_spi_tx)(uint32_t);
    void (*fp_sync_enable)(bool);
    DACXX65_BIT_t datawidth;
} DACXX65_t;

void DACXX65_Init(DACXX65_t *, DACXX65_BIT_t, void (*)(uint32_t), void (*)(bool));
void DACXX65_SetReference(DACXX65_t *, uint16_t);
void DACXX65_WriteChannel(DACXX65_t *, uint8_t, uint16_t);
void DACXX65_SetChannelPower(DACXX65_t *, uint8_t, uint16_t);

#endif
