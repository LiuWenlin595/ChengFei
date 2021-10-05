#pragma once

#include "AircraftTacticalSdk.h"
#include <string>
#include <cmath>

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
	�������࣬�������֯AIģ����TAC-SDK�ӿڵĲο���ơ�
	���������ƣ����԰�AIģ�͵�ҵ���߼���TAC-SDK�Ľӿ���Ч���
*/

// ʵ�ʵ�AIģ�͵���
// AIģ�͵�ʵ��ҵ���߼�ʵ�֡�
// Demo��ʵ��ʵ�ʵ�AI�㷨���߼�ֱ����MyAiModelWrapper�д���
class MyAiModel {
public:
	void init() {};

};

// Ϊ�˶Խ�ƽ̨, ��ʵ�ʵ�AIģ������а�װ.
// Ҳ����ֱ���޸�ԭ��AIģ��,����TacSdkEntity������.
class MyAiModelWrapper :public MyAiModel {
public:
	TacSdkEntity *tac_entity; // TAC-SDK�ӿڶ���
	TacSdkFriend self; // ��������
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
	const int LOOPTIME = 1;

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

	// �ϱ��Էɵ��߼�ʵ�֡�
	// ����������ϣ����ǿ��԰ѵ���̬������Ϊ˫�����������
	// ������ͨ���Էɣ�����ʵ�ֽӽ����״﷢�ֵ�ģ�⡣
	void flat_flight(TacSdkSituationUpdate *situation)
	{
		// ����TAC-SDK�ṩ�Ĺ켣׷�ٽӿڣ�ʵ�ֶԷɡ�
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
		float x = 36875, y = -122007.7;
		float lon = situation->self.base.dof.lon*1000;
		float lat = situation->self.base.dof.lat*1000;
		float height = situation->self.base.dof.height;
		float reward = pow((lon - y), 2) + pow((lat - x), 2);
		env->set_reward(-sqrt(reward));

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	void step(TacSdkSituationUpdate *situation, uint64_t tick) {
		// ����TAC_SDK�����̬���еı�������
		static int cnt = 0; // ���
		self = situation->self;

		// �����״�
		turn_on_radar(tick);

		//clock_t start, end;
		if (is_redgrp()) {
			/*�췽 - ������*/
			//if (tick % 500 == 0) 
				//std::cout << tick << std::endl;

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

				socket.recv(request, zmq::recv_flags::none);	// ����reset
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
			socket.recv(request, zmq::recv_flags::none);	// ����action
			action.ParseFromString(request.to_string());

			// take action
			Action_TrajPoint point = action.point();
			TacSdkTrajPoint p; 
			p.lon = situation->self.base.dof.lon + point.lon();
			p.lat = situation->self.base.dof.lat + point.lat();
			p.ref_phi = situation->self.base.dof.phi;
			p.ref_phi = 0.0;
			p.h = 5600;
			p.vel = 250;
	
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
			/*���� - ���ط�*/
			// ����TAC-SDK�ṩ�Ĺ켣׷�ٽӿڣ�ʵ�ֶԷɡ�
			// ����Χ�Ʒ������򻷷ɣ�ע�⾭γ����ͳ��ȵĵ�λ �� red�췽��γ����
			float goal_lat = 37.28;	// ����Ŀ������lat 37.30183959149617
			float goal_lon = -121.63;	// ����Ŀ������lon -121.654
			float goal_r = 0.01;		// ���ػ��ɰ뾶 r = 0.01
			float goal_R = 0.04;		// �������׷���뾶 R = 0.04
			
			float blue_lat =1, blue_lon;
			float blue_r = 0.04;		// �������ظ�֪��Χ
			float red_lat = situation->self.base.dof.lat;
			float red_lon = situation->self.base.dof.lon;

			float theta, theta_;
			float dis_bg, dis_br;
			
			TacSdkTrajPoint p;
			p.ref_phi = situation->self.base.dof.phi;
			p.ref_phi = 0.0;
			p.h = 5500;
			p.vel = 250;
			
			// ��������
			blue_lat = situation->self.base.dof.lat;
			blue_lon = situation->self.base.dof.lon;
			dis_bg = sqrt(pow(blue_lat - goal_lat, 2) + pow(blue_lon - goal_lon, 2));
			dis_br = sqrt(pow(blue_lat - red_lat, 2) + pow(blue_lon - red_lon, 2)); // red_lat �� red_lon
			
			// �������ع���
			if (dis_br <= blue_r & dis_bg <= goal_R) {	// �췽�����������ظ�֪��Χ
				p.lat = red_lat;
				p.lon = red_lon;

				tac_entity->TrajPointSet(tac_entity, &p);
				trjpoint_set = true;
			
			}
			else {	// �췽δ�����������ظ�֪��Χ
				theta = atan2(blue_lat - goal_lat, blue_lon - goal_lon);
				if (blue_lon < goal_lon) {
					theta += acos(-1);
				}
				theta_ = (theta + acos(-1) / 12);
				if (theta_ >= 2 * acos(-1))
				{
					theta_ = (theta_ - 2 * acos(-1));
				}
			
				p.lon = goal_r * cos(theta_) + blue_lon;
				p.lat = goal_r * sin(theta_) + blue_lat;

				tac_entity->TrajPointSet(tac_entity, &p);
				trjpoint_set = true;
			}
			
		}
		// ƽ��
		//flat_flight(situation);

		// attack
		//attack(situation);
	};

	// 1v1����
	void attack(TacSdkSituationUpdate *situation)
	{
		static int last_num = -1;
		if (!is_redgrp()) {
			//ģ��췽��������
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
		//ģ���ж��Ƿ������Ч�������롣
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

	
	// ��ʾ�����ѵ������ͨ�š�
	// AIģ�����Ҫ�������ݵ�ѵ�����򣬿��������Ƶ��á�

	
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

	// ���ֵл��Ĵ���
	void target_found(TacSdkTargetList *tgt) {
		if (target_reported) {
			return;
		}
		if (!is_redgrp()) {
			//ģ��췽��������
			return;
		}
		// ���ּ򵥹���
		if (tgt->num_enemies > 0 && !target_reported) {
			//std::cout << "found target " << "num:" << tgt->num_enemies << std::endl;
			target_reported = true;
		}
		
		// ѡ��Ŀ��͹�������
		for (int i = 0; i < tgt->num_enemies; i++) {
			uint16_t enemyid = tgt->enemies[i].id.carrier_id;
			tac_entity->TargetSelect(tac_entity, true, enemyid,
				self.wpn_list[0].wpn_id);
		}
	}

	// ������в�Ĵ���
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

// Ϊ�˶Խ�ƽ̨, Ҫ���ݵ�ʵ��ָ��.
typedef struct {
	TacSdkAi tak_sdk;
	MyAiModelWrapper ai_wrapper;
}AiModelInstance;
