#ifndef ROTARY_H
#define ROTARY_H

#include "task.h"

#define ROTARY_TYPE 2 // 2  1:EC11

// #define ROTARY_DEBUG
#ifdef ROTARY_DEBUG
#warning "ROTARY_DEBUG"
#endif // ROTARY_DEBUG

extern uint8_t SYSKey;
extern bool Counter_LOCK_Flag;

void rotary_init(void);
void sys_Counter_Set(double min, double max, double step, double c);
void sys_Counter_SetVal(double c);
void RotaryUp(void);
void RotaryDown(void);

void sys_Counter_click(void);
void sys_Counter_doubleclick(void);
void sys_Counter_longclick(void);
void sys_Counter_IRQHandler(void);
double sys_Counter_Get(void);
uint8_t sys_Counter_Change(void);

void Clear_RButton_FIFO(void);
uint8_t sys_KeyProcess(void);
#endif // ROTARY_H