#ifndef __MAINWORK_H
#define __MAINWORK_H

#include "main.h"
#include "cmsis_os.h"

void main_work(void);

typedef enum 
{
down_location=0,
middle_location=1,
up_location=2	,
pick_middle_location=3
}upper_location;

#endif
