#pragma once

#include "AircraftTacticalSdk.h"
#include <string>

#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>

#include "aimodel.pb.h"
#include <zmq.hpp>

using namespace std::chrono_literals;
using std::string;
extern "C" __declspec(dllexport) TacSdkAi* CreateAiInstance();

/*
	MyAiModel MyAiModelWrapper
	这两个类，是如何组织AI模型与TAC-SDK接口的参考设计。
	采用这个设计，可以把AI模型的业务逻辑与TAC-SDK的接口有效解耦。
*/

// 实际的AI模型的类
// AI模型的实际业务逻辑实现。
// Demo不实现实际的AI算法，逻辑直接在MyAiModelWrapper中处理。
class MyAiModel {
public:
	void init() {};

};

// 为了对接平台, 对实际的AI模型类进行包装.
// 也可以直接修改原有AI模型,增加TacSdkEntity数属性.
class MyAiModelWrapper :public MyAiModel {
public:
	TacSdkEntity *tac_entity; // TAC-SDK接口对象
	TacSdkFriend self; // 本机数据
	bool radar_on;
	bool trjpoint_set;
	bool threat_reported;
	bool target_reported;
	bool target_attacked;

	TacThreatingMissile* g_threat;
	bool g_done;

	bool FIRST;
	zmq::context_t context;
	zmq::socket_t socket;
	const std::string send_msg;
	zmq::pollitem_t items[1];

	clock_t last_time, time;
	const int LOOPTIME = 0;

	MyAiModelWrapper():send_msg("World") {
		FIRST = true;
		g_done = false;

		radar_on = false;
		trjpoint_set = false;
		threat_reported = false;
		target_reported = false;
		target_attacked = false;
		last_time = (clock_t)0;
	}

	void init() {
	}

	bool is_redgrp() {
		return self.base.id.grp_id == 0;
	}

	// 南北对飞的逻辑实现。
	// 在任务设计上，我们可以把敌我态势设置为双方经度相近。
	// 这样，通过对飞，可以实现接近、雷达发现的模拟。
	void flat_flight(TacSdkSituationUpdate *situation)
	{
		// 调用TAC-SDK提供的轨迹追踪接口，实现对飞。
		TacSdkTrajPoint p;
		p.h = 5500;
		p.lon = situation->self.base.dof.lon;
		p.ref_phi = situation->self.base.dof.phi;
		p.ref_phi = 0.0;
		if (situation->self.base.dof.psi > 100) {
			p.lat = situation->self.base.dof.lat-0.1;
			p.vel = 250;
		}
		else {
			p.lat = situation->self.base.dof.lat + 0.1;
			p.vel = 250;
		}
		tac_entity->TrajPointSet(tac_entity, &p);
		trjpoint_set = true;
	}

	Env_Entity* base2entity(TacSdkEntityBaseInfo base) {
		Env_Entity_Dof* dof = new Env_Entity_Dof();
		dof->set_lat(base.dof.lat);
		dof->set_lon(base.dof.lon);
		dof->set_height(base.dof.height);
		dof->set_phi(base.dof.phi);
		dof->set_theta(base.dof.theta);
		dof->set_psi(base.dof.psi);

		Env_Entity_Velocity3D* vel = new Env_Entity_Velocity3D();
		vel->set_vel_north(base.vel.vel_north);
		vel->set_vel_east(base.vel.vel_east);
		vel->set_vel_down(base.vel.vel_down);

		Env_Entity* self = new Env_Entity();
		self->set_allocated_dof(dof);
		self->set_allocated_vel(vel);
		self->set_id(base.id.carrier_id);

		return self;
	}

	Env* get_state_env(TacSdkSituationUpdate *situation) {
		Env* env = new Env();
		Env_Entity* self = base2entity(situation->self.base);
		env->set_allocated_self(self);
		env->set_num_wpn(situation->self.num_wpn);

		// enemy: target_list / situation.enemy
		// missle: threat
		if (situation->target_list.num_enemies > 0) {
			Env_Entity* enemy = base2entity(situation->target_list.enemies[0]);
			env->set_allocated_enemy(enemy);
		}

		if (threat_reported) {
			Env_Missle* missle = new Env_Missle();
			missle->set_dir(g_threat->missile_dir);
			missle->set_dist(g_threat->missile_dist);
		}
		return env;
	}
	void send_state(TacSdkSituationUpdate *situation) {
		Env* env = get_state_env(situation);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	void send_step(TacSdkSituationUpdate *situation) {
		Env* env = get_state_env(situation);

		// reward, done
		env->set_done(g_done);
		env->set_reward(0);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	void step(TacSdkSituationUpdate *situation, uint64_t tick) {
		// 保存TAC_SDK传入的态势中的本机数据
		static int cnt = 0; // 间隔
		self = situation->self;

		// 开启雷达
		turn_on_radar(tick);

		//clock_t start, end;
		if (is_redgrp()) {
			/*红方 - 攻击方*/
			//if (tick % 500 == 0) 
				std::cout << tick << std::endl;

			if (cnt > 0) {
				cnt--;
				return;
			}

			Action action;
			zmq::message_t request;

			if (FIRST) {
				//std::cout << "first" << std::endl;
				context = zmq::context_t{ 1 };
				socket = zmq::socket_t{ context, zmq::socket_type::rep };
				items[0] = { socket, 0, ZMQ_POLLIN, 0 };
				socket.bind("tcp://*:5555");
				std::cout << "bind tcp://*:5555" << std::endl;

				socket.recv(request, zmq::recv_flags::none);	// 接收reset
				action.ParseFromString(request.to_string());

				std::cout << "reset send state" << std::endl;
				send_state(situation);
				FIRST = false;
			}
			else {
				send_step(situation);
			}

			////zmq_poll(items, 1, 1);
			//if (items[0].revents & ZMQ_POLLIN) {
			socket.recv(request, zmq::recv_flags::none);	// 接收action
			action.ParseFromString(request.to_string());

			// take action
			Action_TrajPoint point = action.point();
			TacSdkTrajPoint p;
			p.lat = point.lat();
			p.lon = point.lon();
			p.h = point.h();
			p.vel = point.vel();
			p.ref_phi = point.ref_phi();
			tac_entity->TrajPointSet(tac_entity, &p);

			if (action.deploy() && self.num_wpn > 0) {
				tac_entity->Deploy(tac_entity,
					self.wpn_list[0].wpn_id,
					situation->target_list.enemies[0].id.carrier_id,
					NULL);
			}
			cnt = LOOPTIME;
		}
		else {
			/*蓝方 - 防守方*/
		}
		
		// 平飞
		//flat_flight(situation);

		// attack
		//attack(situation);
	};

	// 1v1攻击
	void attack(TacSdkSituationUpdate *situation)
	{
		static int last_num = -1;
		if (!is_redgrp()) {
			//模拟红方攻击蓝方
			return;
		}
		if (target_attacked) {
			return;
		}

		if (situation->target_list.num_enemies == 0) {
			return;
		}
		if (situation->self.num_wpn == 0) {
			return;
		}
		printf("enemies pos:%lf\n", situation->enemies[0].base.dof.lat);
		//模拟判断是否进入有效攻击距离。
		double dist = abs(self.base.dof.lat - situation->enemies[0].base.dof.lat);
		if(dist<0.3){
			//std::cout << "deploy" << std::endl;
			tac_entity->Deploy(tac_entity, 
				self.wpn_list[0].wpn_id, 
				situation->target_list.enemies[0].id.carrier_id,
				NULL);
			target_attacked = true;
		}
	}

	
	// 演示如何与训练程序通信。
	// AI模型如果要发送数据到训练程序，可以做类似调用。

	
	//TacSdkAiRet notify_combat_finished() {
	void notify_combat_finished() {
		//char msg[256] = { 0 };
		//snprintf(msg, 256, "AI:%d detected dead ac.", self.base.id.carrier_id);
		//tac_entity->SendSchedulerMsg(tac_entity, (void *)msg, strlen(msg) + 1);
		g_done = true;
		//return TSAIRET_HALT;
	}

	void turn_on_radar(uint64_t tick) {
		if (radar_on == true) 
			return;
		//if (tick < 300) {
		//	return;
		//}
		uint8_t on = 1;
		tac_entity->SensorControl(tac_entity, 0,  TACSDK_SENSOR_CMD_ONOFF, (void *)&on, sizeof(uint8_t));
		radar_on = true;
	}

	// 发现敌机的处理
	void target_found(TacSdkTargetList *tgt) {
		if (target_reported) {
			return;
		}
		if (!is_redgrp()) {
			//模拟红方攻击蓝方
			return;
		}
		// 发现简单攻击
		if (tgt->num_enemies > 0 && !target_reported) {
			//std::cout << "found target " << "num:" << tgt->num_enemies << std::endl;
			target_reported = true;
		}
		
		// 选定目标和攻击武器
		for (int i = 0; i < tgt->num_enemies; i++) {
			uint16_t enemyid = tgt->enemies[i].id.carrier_id;
			tac_entity->TargetSelect(tac_entity, true, enemyid,
				self.wpn_list[0].wpn_id);
		}
	}

	// 发现威胁的处理
	void threat_detected(TacThreatingMissile* missile) {
		//if (threat_reported) {
		//	return;
		//}
		//std::cout << "threat" << std::endl;
		//std::cout << "id: " << missile->missile_id << std::endl << "dist:" << missile->missile_dist << std::endl << "dir:" << missile->missile_dir << std::endl;
		g_threat = missile;
		threat_reported = true;
	}
};

// 为了对接平台, 要传递的实例指针.
typedef struct {
	TacSdkAi tak_sdk;
	MyAiModelWrapper ai_wrapper;
}AiModelInstance;
