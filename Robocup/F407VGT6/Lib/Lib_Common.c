#include "Lib_Common.h"
#include "task.h"

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void* malloc_wrapper(size_t size)
{
    return pvPortMalloc(size);
}

void free_wrapper(void *pointer)
{
    vPortFree(pointer);
}