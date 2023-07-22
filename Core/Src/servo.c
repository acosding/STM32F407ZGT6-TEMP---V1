#include "servo.h"

void servo(int num, float degree){
	int pwmout = 0;
	if((num<1)||(num>4)){
		return;
	}
	if((degree<-90)||(degree>90)){
		return;
	}
	pwmout = 1500 + 1000*(degree/90);
	switch (num){
		case 1 :
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pwmout);
			break;
		case 2 :
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,pwmout);
			break;
		case 3 :
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,pwmout);
			break;
		case 4:
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,pwmout);
			break;
	}
}
