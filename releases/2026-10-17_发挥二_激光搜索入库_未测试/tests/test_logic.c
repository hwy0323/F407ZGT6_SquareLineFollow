#include <assert.h>
#include <stdio.h>
#include "../entry_plan.h"
int main(void)
{
  uint16_t samples[5],median;unsigned checks=0;
  for(unsigned mm=20;mm<=500;mm++){
    for(unsigned i=0;i<5;i++)samples[i]=mm;
    GarageType expected=(mm>=101&&mm<=151)?GARAGE_SIDE:(mm>=201&&mm<=251)?GARAGE_REVERSE:GARAGE_UNKNOWN;
    assert(search_classify(samples,74,&median)==expected);checks++;
  }
  uint16_t noisy[5]={126,127,126,170,125};
  assert(search_classify(noisy,74,&median)==GARAGE_UNKNOWN);
  noisy[3]=0;assert(search_classify(noisy,74,&median)==GARAGE_UNKNOWN);
  assert(search_classify(noisy,-1,&median)==GARAGE_UNKNOWN);
  for(uint8_t edge=0;edge<4;edge++){
    int32_t p=edge?search_corner_counts[edge-1]+2500:1500;
    assert(search_feature(0x18,edge,p)==FEATURE_LINE);
    assert(search_feature(0x08,edge,p)==FEATURE_LINE);
    assert(search_feature(0x10,edge,p)==FEATURE_LINE);
    assert(search_feature(0x38,edge,p)==FEATURE_LINE);
    assert(search_feature(0x1C,edge,p)==FEATURE_LINE);
    assert(search_feature(0x0F,edge,p)==FEATURE_GARAGE);
    assert(search_feature(0xFF,edge,p)==FEATURE_GARAGE);
    assert(search_feature(0x0F,edge,search_corner_counts[edge])==FEATURE_CORNER);
    assert(search_feature(0xFF,edge,search_corner_counts[edge])==FEATURE_AMBIGUOUS);
    assert(search_feature(0x18,edge,search_corner_counts[edge]+901)==FEATURE_AMBIGUOUS);
  }
  assert(search_feature(0x0F,4,30000)==FEATURE_END);
  search_plan_reset();assert(reference_count==1 && reference_steps[0].target==287);
  assert(!search_plan_return(246) && !search_plan_return(328));
  assert(search_plan_return(300) && reference_steps[1].target==300);
  assert(!search_plan_entry(GARAGE_UNKNOWN));
  assert(search_plan_entry(GARAGE_SIDE) && reference_count==7 && reference_steps[5].target==1347);
  assert(search_plan_entry(GARAGE_REVERSE) && reference_count==5 && reference_steps[3].target==9000);
  printf("PASS LOGIC: %u range values plus noise, limits, four-edge features and plans\n",checks);
}
