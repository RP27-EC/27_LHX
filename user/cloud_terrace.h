#ifndef CLOUD_TERRACE_H
#define CLOUD_TERRACE_H

/* 云台状态仅供调试观察；控制入口由固定周期任务调用。 */
typedef enum
{
    CLOUD_TERRACE_HOME_WAIT = 0,
    CLOUD_TERRACE_HOME_MOVING,
    CLOUD_TERRACE_HOME_DONE
} CloudTerrace_HomeState_t;

extern volatile CloudTerrace_HomeState_t cloud_terrace_home_state;

void CloudTerrace_Init(void);
void CloudTerrace_Update(void);

#endif /* CLOUD_TERRACE_H */
