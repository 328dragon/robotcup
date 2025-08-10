#ifndef __CHASSISLOGIC_H
#define __CHASSISLOGIC_H


#define SHOVOL 0.3
typedef enum
{
    LEAVE_HOME = 1,//离开家
    FIND_GRAY,//等待灰度寻找十字
    FIND_THING,//前往寻找物体
    WAITPICK,//等待上层机构拾取物体
    TOGOAL,//开环粗前往目标点
    FINDING_GOAL,//等待摄像头找到目标点
    FINDED_GOAL,//摄像头找到目标点
    TO_ADJUSTGOAL,//前往并调整目标点
    WAITGOAL//等待上层机构得分

}ChassisTaskFlag;


#endif
