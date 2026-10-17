#ifndef FAKE_PWM_HAL_H
#define FAKE_PWM_HAL_H
#include <stdint.h>
#include <assert.h>
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t OCMode,Pulse,OCPolarity,OCFastMode;} TIM_OC_InitTypeDef;
typedef struct {uint32_t Prescaler,CounterMode,Period,ClockDivision,AutoReloadPreload;} TimerInit;
typedef struct {void *Instance;TimerInit Init;uint32_t c3,c4;} TIM_HandleTypeDef;
#define GPIOA ((void*)1)
#define TIM5 ((void*)5)
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_MODE_AF_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 2U
#define GPIO_AF2_TIM5 2U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1U
#define TIM_OCMODE_PWM1 1U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_CHANNEL_3 3U
#define TIM_CHANNEL_4 4U
#define HAL_OK 0
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM5_CLK_ENABLE() ((void)0)
static uint8_t fake_fail,fake_running3,fake_running4;
static void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *p){
  assert(port==GPIOA && p->Pin==(GPIO_PIN_2|GPIO_PIN_3) && p->Alternate==GPIO_AF2_TIM5);
}
static int HAL_TIM_PWM_Init(TIM_HandleTypeDef *t){assert(t->Instance==TIM5);return fake_fail;}
static int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *t,TIM_OC_InitTypeDef *p,uint32_t channel){
  assert(p->Pulse==0);if(channel==3)t->c3=p->Pulse;else{assert(channel==4);t->c4=p->Pulse;}return 0;
}
static int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c){
  (void)t;if(c==3)fake_running3=1;else{assert(c==4);fake_running4=1;}return 0;
}
static void fake_compare(TIM_HandleTypeDef *t,uint32_t c,uint32_t pulse){
  assert(pulse>=500 && pulse<=2500);if(c==3)t->c3=pulse;else{assert(c==4);t->c4=pulse;}
}
#define __HAL_TIM_SET_COMPARE(timer,channel,pulse) fake_compare(timer,channel,pulse)
#endif
