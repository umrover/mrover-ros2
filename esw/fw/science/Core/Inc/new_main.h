#ifndef __NEW_MAIN_H
#define __NEW_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void new_main(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#ifdef __cplusplus
}
#endif

#endif
