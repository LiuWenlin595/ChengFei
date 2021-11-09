#include <stdlib.h>
#include <memory.h>
#include "demodll.h"
#include "stdio.h"


//Ҫ����DFSƽ̨�ļ�������, ����TacSdkAi�ṹ��
TacSdkAiRet _config(TacSdkAi *ai, TacSdkEntity * entity, const char *pnames[], const char *pvalues[], uint16_t num_params)
{
	AiModelInstance *ins = (AiModelInstance*)ai;
	//��TacSdkEntity������
	ins->ai_wrapper.tac_entity = entity;

	//����AIģ����Ҫ, ��ʼ��AIģ��
	ins->ai_wrapper.init();
	return TSAIRET_SUCCESS;
}

TacSdkAiRet _step(TacSdkAi *ai, uint64_t tick_cnt, double interval, TacSdkSituationUpdate * situation)
{
	AiModelInstance *ins = (AiModelInstance*)ai;

	// ��ʾAIģ������˳�һ��ս����
	// AIģ������жϸ���ѵ�����Խ���������ͨ���˳����̵ķ�ʽ�����С�
	// ƽ̨�Ὣexit��ֵ��ͨ���ص������ݸ�ѵ������
	// ����һ����ʽ����step�з���HALT��
	if (tick_cnt == 600000) {
		exit(123);
	}
	
	// ����, done��ΪTrue
	if (!situation->self.base.is_alive) {
		ins->ai_wrapper.notify_combat_finished();
	}

	//����TacSdkSituationUpdate����,����Ӧ����.
	ins->ai_wrapper.step(situation, tick_cnt);
	// std::ofstream out("D:\\Project\\out.txt", std::ios::app);
	// out << "step done ..." << std::endl;
	return TSAIRET_SUCCESS;
}

TacSdkAiRet _procEvent(TacSdkAi *ai, TacSdkAiEvent * event) {
	AiModelInstance *ins = (AiModelInstance*)ai;
	if (event->header.type == TACSDK_EVT_TAGET_LIST)
	{
		// ���ֵл�Ŀ��Ĵ���
		TacSdkTargetList *tgtlist = (TacSdkTargetList *) event->body;
		ins->ai_wrapper.target_found(tgtlist);
	}
	else if (event->header.type == TACSDK_EVT_WARNING)
	{
		// ���ָ澯�Ĵ���
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

	// AIģ�Ϳ��Ը���TAC-SDK״̬�仯�������ܷ�������������������
}

void _reset(TacSdkAi *ai)
{
	// AIģ�����ýӿ�
}

void _clean(TacSdkAi *ai) 
{ 
	// AIģ������ӿ�
}

// AI ģ�Ͷ�̬�⵼����Ψһ�ӿں���
TacSdkAi* CreateAiInstance()
{
	AiModelInstance *ins = (AiModelInstance*) malloc(sizeof(AiModelInstance));
	ins->tak_sdk.Clean = _clean;
	ins->tak_sdk.Config = _config;
	ins->tak_sdk.ProcEvent = _procEvent;
	ins->tak_sdk.Reset = _reset;
	ins->tak_sdk.StateChange= _stateChange;
	ins->tak_sdk.Step = _step;

	// ��ʽ����MyAiModelWrapper
	char *p = (char *) ins;
	p = p + sizeof(TacSdkAi);
	new(p) MyAiModelWrapper();
	return (TacSdkAi *)ins;
}


