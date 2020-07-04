#pragma once
#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define	USE_DEBUG_PRINTF
//#define	USE_DEFAULT_SETTINGS

#define I2C_EEPROM_PAGE_SIZE		64

typedef enum
{
	CRC_ERROR = -1,
	DATA = 0,
	END_OF_FILE = 1,
	SEGMENT_ADDR = 2,
	START_SEGMENT_ADDR_REC = 3,
	EXTENDED_ADDR = 4,
	START_LINEAR_ADDR_REC = 5
} hex_rec_type_t;

//~ Main settings ~//
#pragma pack (push, 1)

typedef struct
{
	unsigned char writing_tag_begin;
	boot_state_t FlashUpdateFlag;
	unsigned int SizeOfFirmware;
	unsigned char writing_tag_end;

} Settings_t;

#pragma pack (pop)
//~ Main settings ~//

extern Settings_t Settings;
;
void Setting_Save (void);
void Settings_Init (bool reset_to_default);

int i2c_eeprom_erase (void);
int i2c_eeprom_write_page (unsigned short PageAddress, unsigned char *Data);
int i2c_eeprom_write (unsigned short MemAddress, unsigned char *pData, unsigned short Size);
int i2c_eeprom_read (unsigned short MemAddress, unsigned char *pData, unsigned short Size);

#ifdef __cplusplus
}
#endif

#endif
