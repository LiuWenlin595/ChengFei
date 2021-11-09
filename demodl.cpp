#include <stdlib.h>
#include <memory.h>
#include "demodll.h"
#include "stdio.h"


//要传给DFS平台的几个函数, 放入TacSdkAi结构体
TacSdkAiRet _config(TacSdkAi *ai, TacSdkEntity * entity, const char *pnames[], const char *pvalues[], uint16_t num_params)
{
	AiModelInstance *ins = (AiModelInstance*)ai;
	//与TacSdkEntity建立绑定
	ins->ai_wrapper.tac_entity = entity;

	//根据AI模型需要, 初始化AI模型
	ins->ai_wrapper.init();
	return TSAIRET_SUCCESS;
}

TacSdkAiRet _step(TacSdkAi *ai, uint64_t tick_cnt, double interval, TacSdkSituationUpdate * situation)
{
	AiModelInstance *ins = (AiModelInstance*)ai;

	// 演示AI模型如何退出一个战场。
	// AI模型如果判断该轮训练可以结束，可以通过退出进程的方式来进行。
	// 平台会将exit的值，通过回调，传递给训练程序。
	// 另外一个方式是在step中返回HALT。
	if (tick_cnt == 600000) {
		exit(123);
	}
	
	// 死亡, done设为True
	if (!situation->self.base.is_alive) {
		ins->ai_wrapper.notify_combat_finished();
	}

	//根据TacSdkSituationUpdate内容,做相应处理.
	ins->ai_wrapper.step(situation, tick_cnt);
	// std::ofstream out("D:\\Project\\out.txt", std::ios::app);
	// out << "step done ..." << std::endl;
	return TSAIRET_SUCCESS;
}

TacSdkAiRet _procEvent(TacSdkAi *ai, TacSdkAiEvent * event) {
	AiModelInstance *ins = (AiModelInstance*)ai;
	if (event->header.type == TACSDK_EVT_TAGET_LIST)
	{
		// 发现敌机目标的处理
		TacSdkTargetList *tgtlist = (TacSdkTargetList *) event->body;
		ins->ai_wrapper.target_found(tgtlist);
	}
	else if (event->header.type == TACSDK_EVT_WARNING)
	{
		// 发现告警的处理
		TacThreatingMissile *threat = (TacThreatingMissile *)event->body;
		ins->ai_wrapper.threat_detected(threat);
	}
	return TSAIRET_SUCCESS;
}

void _stateChange(TacSdkAi *ai, TacState new_state) 
{ 
	//	typedef uint16_t TacState;
	//#define TSS_INACTIVE	0		// TAC-SDK is invactive, will only response inter-comm
	//#define TSS_SEMI_ACTIVE	1		// TAC-SDK is semi-active, will response sensor, weapon and intercomm
	//#define TSS_ACTIVE		2		// TAC-SDK is active, will reponse flight control, sensor weapon and intercomm

	// AI模型可以根据TAC-SDK状态变化，决定能否启动武器及传感器。
}

void _reset(TacSdkAi *ai)
{
	// AI模型重置接口
}

void _clean(TacSdkAi *ai) 
{ 
	// AI模型清理接口
}

// AI 模型动态库导出的唯一接口函数
TacSdkAi* CreateAiInstance()
{
	AiModelInstance *ins = (AiModelInstance*) malloc(sizeof(AiModelInstance));
	ins->tak_sdk.Clean = _clean;
	ins->tak_sdk.Config = _config;
	ins->tak_sdk.ProcEvent = _procEvent;
	ins->tak_sdk.Reset = _reset;
	ins->tak_sdk.StateChange= _stateChange;
	ins->tak_sdk.Step = _step;

	// 显式构造MyAiModelWrapper
	char *p = (char *) ins;
	p = p + sizeof(TacSdkAi);
	new(p) MyAiModelWrapper();
	return (TacSdkAi *)ins;
}


