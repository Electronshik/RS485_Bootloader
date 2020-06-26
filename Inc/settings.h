#pragma once
#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define	USE_DEBUG_PRINTF
//#define	USE_DEBUG_IP
//#define	USE_DEFAULT_SETTINGS
#define	USE_UDP_REMOTE_IP

#define I2C_EEPROM_PAGE_SIZE		64

//#include "stdbool.h"

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
	unsigned char Main_Led_Value;
	unsigned char Mon_IR_Value;
	unsigned char writing_tag_end;

} Settings_t;

#pragma pack (pop)
//~ Main settings ~//

//extern I2C_HandleTypeDef hi2c1;

//hex_rec_type_t CheckHexCrc (unsigned char * line);
//unsigned char *GetHexData (unsigned char * line);

int i2c_eeprom_erase (void);
int i2c_eeprom_write_page (unsigned short PageAddress, unsigned char *Data);
int i2c_eeprom_write (unsigned short MemAddress, unsigned char *pData, unsigned short Size);
int i2c_eeprom_read (unsigned short MemAddress, unsigned char *pData, unsigned short Size);

#ifdef __cplusplus
}
#endif

#endif
