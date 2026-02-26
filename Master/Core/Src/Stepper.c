/*
 * Stepper.c
 *
 *  Created on: 24 thg 12, 2025
 *      Author: Admin
 */

#include "Stepper.h"

static const uint8_t step_table[8][4] =
{
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

static void Stepper_Write(Stepper_Handle_t *m,
                          uint8_t a, uint8_t b,
                          uint8_t c, uint8_t d)
{
    HAL_GPIO_WritePin(m->IN1_Port, m->IN1_Pin, a);
    HAL_GPIO_WritePin(m->IN2_Port, m->IN2_Pin, b);
    HAL_GPIO_WritePin(m->IN3_Port, m->IN3_Pin, c);
    HAL_GPIO_WritePin(m->IN4_Port, m->IN4_Pin, d);
}

void Stepper_Init(Stepper_Handle_t *motor,
                  GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                  GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                  GPIO_TypeDef *IN3_Port, uint16_t IN3_Pin,
                  GPIO_TypeDef *IN4_Port, uint16_t IN4_Pin,
                  uint16_t steps_per_rev)
{
    /* Lưu mapping chân */
    motor->IN1_Port = IN1_Port;
    motor->IN1_Pin  = IN1_Pin;

    motor->IN2_Port = IN2_Port;
    motor->IN2_Pin  = IN2_Pin;

    motor->IN3_Port = IN3_Port;
    motor->IN3_Pin  = IN3_Pin;

    motor->IN4_Port = IN4_Port;
    motor->IN4_Pin  = IN4_Pin;

    motor->steps_per_rev = steps_per_rev;

    /* Init trạng thái */
    motor->step_index = 0;

    Stepper_Stop(motor);
}

void Stepper_Step(Stepper_Handle_t *motor, int dir)
{
    if (dir > 0)
        motor->step_index = (motor->step_index + 1) % 8;
    else
        motor->step_index = (motor->step_index + 7) % 8;

    Stepper_Write(motor,
        step_table[motor->step_index][0],
        step_table[motor->step_index][1],
        step_table[motor->step_index][2],
        step_table[motor->step_index][3]
    );

    HAL_Delay(2);
}

void Stepper_RotateSteps(Stepper_Handle_t *motor, int steps)
{
    int dir = (steps >= 0) ? 1 : -1;
    steps = (steps >= 0) ? steps : -steps;

    for (int i = 0; i < steps; i++)
        Stepper_Step(motor, dir);
}

void Stepper_RotateAngle(Stepper_Handle_t *motor, float angle_deg)
{
    int steps = (int)((angle_deg * motor->steps_per_rev) / 360.0f);
    Stepper_RotateSteps(motor, steps);
}

void Stepper_Stop(Stepper_Handle_t *motor)
{
    Stepper_Write(motor, 0,0,0,0);
}

