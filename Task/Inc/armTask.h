//
// Created by 20852 on 2026/9/30.
//

#ifndef STARTM3508_ARMTASK_H
#define STARTM3508_ARMTASK_H

#pragma once
#include "../TaskBase.h"

#ifdef __cplusplus
class ArmTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void ArmTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_ARMTASK_H
