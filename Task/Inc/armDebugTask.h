//
// armDebugTask.h — 机械臂标定/调试模式任务
//

#ifndef STARTM3508_ARMDEBUGTASK_H
#define STARTM3508_ARMDEBUGTASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class ArmDebugTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void ArmDebugTask_Init();

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_ARMDEBUGTASK_H
