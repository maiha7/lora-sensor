#ifndef __CRC8_H__
#define __CRC8_H__

extern unsigned char crc_array[256];

unsigned char CRC8_Table(unsigned char *p, char counter);

unsigned char  calcrc_1byte(unsigned char  abyte) ;

unsigned char  calcrc_bytes(unsigned char  *p,unsigned char  len);

uint8_t calc_crc8_x8_x2_x_1(uint8_t code, void *buf, uint8_t len);

uint8_t calc_check_sum8(void *pdata, uint8_t len);

#endif