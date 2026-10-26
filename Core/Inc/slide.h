#ifndef SLIDE_H
#define SLIDE_H

#include "main.h"

void Slide_Init(void);
void Slide_Enable(void);
void Slide_Release(void);
void Slide_Move(int32_t steps);
void Slide_ReturnToStart(void);
void Slide_RunIdle(uint32_t now);
int32_t Slide_GetPosition(void);

#endif /* SLIDE_H */
