#include "search_logic.h"

/* Only true-corner positions are retained. No stored garage positions. */
const int32_t search_corner_counts[4] = {5100,11299,17567,23793};
static uint8_t count_bits(uint8_t b)
{
  uint8_t n=0; while(b){n+=b&1U;b>>=1;} return n;
}
TrackFeature search_feature(uint8_t bits,uint8_t corners,int32_t progress)
{
  if(corners>=4U)return FEATURE_END;
  uint8_t l=count_bits(bits&0xF0U),r=count_bits(bits&0x0FU);
  uint8_t broad=(l>=3U&&(bits&0x80U))||(r>=3U&&(bits&0x01U));
  int32_t error=progress-search_corner_counts[corners];
  if(error>CORNER_WINDOW_COUNTS)return FEATURE_AMBIGUOUS;
  if(!broad)return FEATURE_LINE; /* A single light/three central lights: correct line. */
  if(error>=-CORNER_WINDOW_COUNTS){
    /* Near a physical corner do not guess that a wide tape feature is a bay. */
    return r==4U&&l<=1U ? FEATURE_CORNER : FEATURE_AMBIGUOUS;
  }
  /* Ignore the previous corner's recovery area, without entering a garage. */
  if(corners && progress<search_corner_counts[corners-1U]+CORNER_WINDOW_COUNTS)
    return FEATURE_LINE;
  return FEATURE_GARAGE;
}

GarageType search_classify(const uint16_t samples[RANGE_SAMPLE_COUNT],
                           uint16_t *median_mm)
{
  uint16_t sorted[RANGE_SAMPLE_COUNT];
  *median_mm=0U;
  for(uint8_t i=0;i<RANGE_SAMPLE_COUNT;i++){
    if(samples[i]<20U || samples[i]>2000U)return GARAGE_UNKNOWN;
    sorted[i]=samples[i];
  }
  for(uint8_t i=0;i<RANGE_SAMPLE_COUNT;i++)
    for(uint8_t j=i+1;j<RANGE_SAMPLE_COUNT;j++)
      if(sorted[j]<sorted[i]){uint16_t t=sorted[i];sorted[i]=sorted[j];sorted[j]=t;}
  *median_mm=sorted[RANGE_SAMPLE_COUNT/2U];
  if(sorted[RANGE_SAMPLE_COUNT-1U]-sorted[0]>RANGE_SPREAD_MAX_MM)return GARAGE_UNKNOWN;
  /* 直接比较原始读数；237 mm 归侧停，不加安装偏移量。 */
  return *median_mm<=LASER_TYPE_THRESHOLD_MM ? GARAGE_SIDE : GARAGE_REVERSE;
}
