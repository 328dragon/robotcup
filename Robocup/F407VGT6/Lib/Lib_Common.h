#ifndef __LIB_COMMON_H
#define __LIB_COMMON_H

#include <stdlib.h>
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);
void delay(uint32_t ms);
void* malloc_wrapper(size_t size);
void free_wrapper(void *pointer);

typedef struct {
    void* (*allocate)(size_t size);
    void (*deallocate)(void* ptr);
} CustomAllocator;

#ifdef __cplusplus
}
#endif

#endif