#ifndef MAP_FEATURES_H
#define MAP_FEATURES_H
#include <stdint.h>
static uint8_t map_garage_candidate(uint8_t bits)
{
  uint8_t left=0U,right=0U;
  for(uint8_t i=0U;i<4U;i++){
    left+=(bits>>(i+4U))&1U; right+=(bits>>i)&1U;
  }
  return (left>=3U && (bits&0x80U)) || (right>=3U && (bits&0x01U));
}
#endif
