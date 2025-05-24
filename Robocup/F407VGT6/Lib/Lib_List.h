/*
 * @Author: Elaina
 * @Date: 2024-10-16 22:24:42
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-17 18:25:56
 * @FilePath: \MDK-ARM\Lib\Lib_List.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
/*
 * @Author: Elaina
 * @Date: 2024-10-16 22:24:42
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-17 18:25:22
 * @FilePath: \MDK-ARM\Lib\Lib_Math.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __LIB_LIST_H
#define __LIB_LIST_H
#include "Lib_Common.h"
template <typename T>
class LibList_t
{
public:
    template <typename Func>
    void Foreach(Func func)
    {
        for (int i = 0; i < size; i++)
        {
            func(data[i]); // 调用传入的函数对象
        }
    }
    void Foreach(void (*func)(T &))
    {
        for (int i = 0; i < size; i++)
        {
            func(data[i]);
        }
    }
    T &Find(bool (*func)(T &))
    {
        for (int i = 0; i < size; i++)
        {
            if (func(data[i]))
            {
                return data[i];
            }
        }
        return nullptr;
    }
    void Add(T &&item)
    {
        if (size == 0)
        {
            data = new T[1];
            data[0] = item;
            front_ptr = data;
            tail_ptr = data + 1;
            size = 1;
        }
        else
        {
            T *temp = new T[size + 1];
            for (int i = 0; i < size; i++)
            {
                temp[i] = data[i];
            }
            temp[size] = item;
            delete[] data;
            data = temp;
            size++;
            tail_ptr = data + size;
        }
    }
    T &operator[](int index)
    {
        return data[index];
    }
    T *data;

private:
    T *front_ptr;
    T *tail_ptr;
    int size;
};

#endif
