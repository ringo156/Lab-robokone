/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "Servo.h"
#include "pc_control.h"
#include <stdio.h>

#define NUM 3
#define STRETCH 127
#define SPEED 127

void init();
void init_servo(Servo_Data *servo);
void rad_conv(Servo_Data *servo, float *radian, uint8 val);

int main(void)
{
    uint8 i;
    char buf[30];
    float radian[3] = {0.0, 0.0, 0.0};
    Servo_Data SERVO[3];
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    init();
    init_servo(SERVO);
    initComPc();

    for(;;)
    {
        /* Place your application code here. */
        getRxRadian(radian, 3);
        rad_conv(SERVO, radian, 3);
        for(i = 0; i < NUM ;i++){
            angle_set(&SERVO[i], SERVO[i].angle);
        }
        
    }
}

void init()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_servo_Start();
    UART_debug_Start();
    UART_debug_PutString("Hello PSoC World\n");
}

void init_servo(Servo_Data *servo)
{
    uint8 i;
    int16 max[3] = {90, 90, 90};
    int16 min[3] = {-90, -20, -90};
    for(i = 0; i < NUM; i++){
        Servo_Dataset(&servo[i], i+1, SPEED, STRETCH, 0, max[i], min[i]);
        angle_set(&servo[i], 0);
    }
}

void rad_conv(Servo_Data *servo, float *radian, uint8 val)
{
    uint8 i;
    for(i=0;i<val;i++){
        servo[i].angle = (int16)(radian[i] * 57.29577);
    }
}

/* [] END OF FILE */
