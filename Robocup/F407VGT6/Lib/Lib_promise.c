#include "Lib_pormise.h"

/* 构造函数（初始化函数） */
void SimpleStatus_t_init(SimpleStatus_t* self) {
    self->_isResolved = false;
}

/* 判断是否已解决 */
bool SimpleStatus_t_isResolved(SimpleStatus_t* self) {
    return self->_isResolved;
}

/* 设置为已解决 */
void SimpleStatus_t_resolve(SimpleStatus_t* self) {
    self->_isResolved = true;
}

/* 重置状态 */
void SimpleStatus_t_start(SimpleStatus_t* self) {
    self->_isResolved = false;
}