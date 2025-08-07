#ifndef __LIB_PROMISE_H
#define __LIB_PROMISE_H
#include <stdbool.h>
typedef struct {
    /* 可以添加必要的成员变量 */
} Promise_t;

/* SimpleStatus_t 结构体 */
typedef struct {
    bool _isResolved;
} SimpleStatus_t;

void SimpleStatus_t_init(SimpleStatus_t* self);
bool SimpleStatus_t_isResolved(SimpleStatus_t* self);
void SimpleStatus_t_resolve(SimpleStatus_t* self);
void SimpleStatus_t_start(SimpleStatus_t* self);

#endif