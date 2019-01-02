#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMA   TIM1->CCR1  //PA8
#define PWMB   TIM1->CCR2  //PA9
#define PWMC   TIM1->CCR3  //PA10
#define PWMD   TIM1->CCR4  //PA11
#define CIN1   PBout(12)
#define CIN2   PBout(13)
#define DIN1   PBout(14)
#define DIN2   PBout(15)
#define AIN1   PAout(4)
#define AIN2   PAout(5)
#define BIN1   PAout(6)
#define BIN2   PAout(7)


void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
