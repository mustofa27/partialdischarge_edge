/*
 * types.h — tipe lebar-tetap tanpa libc.
 *
 * Proyek ini dibangun freestanding (-nostdlib): tidak ada newlib, jadi
 * <stdint.h> tidak tersedia. Tipe diambil dari makro bawaan GCC, yang selalu
 * ada dan selalu cocok dengan target yang sedang dikompilasi.
 */

#ifndef PD_TYPES_H
#define PD_TYPES_H

typedef __UINT8_TYPE__  uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef __UINT32_TYPE__ uint32_t;
typedef __INT32_TYPE__  int32_t;

#endif /* PD_TYPES_H */
