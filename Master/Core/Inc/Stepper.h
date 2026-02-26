/*
 * Stepper.h
 *
 *  Created on: 24 thg 12, 2025
 *      Author: Admin
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "stm32f1xx_hal.h"

typedef struct
{
    GPIO_TypeDef *IN1_Port;
    uint16_t      IN1_Pin;

    GPIO_TypeDef *IN2_Port;
    uint16_t      IN2_Pin;

    GPIO_TypeDef *IN3_Port;
    uint16_t      IN3_Pin;

    GPIO_TypeDef *IN4_Port;
    uint16_t      IN4_Pin;

    uint16_t steps_per_rev;   // số step / vòng (ví dụ: 4096)
    uint8_t  step_index;      // vị trí bước hiện tại
} Stepper_Handle_t;

/* ===== API ===== */
void Stepper_Init(Stepper_Handle_t *motor,
                  GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                  GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                  GPIO_TypeDef *IN3_Port, uint16_t IN3_Pin,
                  GPIO_TypeDef *IN4_Port, uint16_t IN4_Pin,
                  uint16_t steps_per_rev);

void Stepper_Step(Stepper_Handle_t *motor, int dir);
void Stepper_RotateSteps(Stepper_Handle_t *motor, int steps);
void Stepper_RotateAngle(Stepper_Handle_t *motor, float angle_deg);
void Stepper_Stop(Stepper_Handle_t *motor);

#endif /* INC_STEPPER_H_ */
