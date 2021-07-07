//
// Created by Felix on 6/24/2021.
//

#include "main.h"
#define PWM_PRD 320000

void PWM_Motor_Update(uint32_t thro1,uint32_t thro2,uint32_t thro3,uint32_t thro4){
  if(thro1>PWM_PRD)thro1 = PWM_PRD;
  if(thro2>PWM_PRD)thro2 = PWM_PRD;
  if(thro3>PWM_PRD)thro3 = PWM_PRD;
  if(thro4>PWM_PRD)thro4 = PWM_PRD;

  if(thro1<=0) thro1 = 0;
  if(thro2<=0) thro2 = 0;
  if(thro3<=0) thro3 = 0;
  if(thro4<=0) thro4 = 0;

  TIM2->CCR1 = thro1;
  TIM2->CCR2 = thro2;
  TIM2->CCR3 = thro3;
  TIM2->CCR4 = thro4;

}