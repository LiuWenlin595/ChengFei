#pragma once

#include "AircraftTacticalSdk.h"
#include <string>
#include <cmath>
#include <random>
#include <ctime>

#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>

#include "aimodel.pb.h"
#include <zmq.hpp>

using namespace std::chrono_literals;
using std::string;
extern "C" __declspec(dllexport) TacSdkAi *CreateAiInstance();

/*
	MyAiModel MyAiModelWrapper
	�������࣬�������֯AIģ����TAC-SDK�ӿڵĲο���ơ�
	���������ƣ����԰�AIģ�͵�ҵ���߼���TAC-SDK�Ľӿ���Ч���
*/

// ʵ�ʵ�AIģ�͵���
// AIģ�͵�ʵ��ҵ���߼�ʵ�֡�
// Demo��ʵ��ʵ�ʵ�AI�㷨���߼�ֱ����MyAiModelWrapper�д���
class MyAiModel
{
public:
	void init(){};
};

// Ϊ�˶Խ�ƽ̨, ��ʵ�ʵ�AIģ������а�װ.
// Ҳ����ֱ���޸�ԭ��AIģ��,����TacSdkEntity������.
class MyAiModelWrapper : public MyAiModel
{
public:
	TacSdkEntity *tac_entity; // TAC-SDK�ӿڶ���
	TacSdkFriend self;		  // ��������
	bool radar_on;			  // �״��Ƿ��
	bool trjpoint_set;
	bool threat_reported; // �Ƿ��ֵ���
	bool target_reported; // �Ƿ��ֵл�
	bool target_attacked; // �Ƿ��������

	TacThreatingMissile g_threat; // ������Ϣ
	bool red_crash;				// �췽�Ƿ񱻻���
	bool blue_crash;			// �����Ƿ񱻻���
	//int periodCount;	// ����ļ��ж��Ƿ�C++������

	bool FIRST; // �Ƿ��ǿ��ֵ�һ֡
	zmq::context_t context;
	zmq::socket_t socket;
	const std::string send_msg;
	zmq::pollitem_t items[1];

	clock_t last_time, time;
	const int LOOPTIME = 1;

	const float BlueX = -121.5;
	const float BlueY = 37;

	std::default_random_engine e;
	std::uniform_real_distribution<float> u;
	float deltax, deltay, g_goal_x, g_goal_y; // xĿ��㾭��, yĿ���γ��

	float GoalRadius = 0.1; // ���ػ��ɰ뾶 r = 0.1

	MyAiModelWrapper() : send_msg("World")
	{
		FIRST = true;
		red_crash = false;
		blue_crash = false;

		radar_on = false;
		trjpoint_set = false;
		threat_reported = false;
		target_reported = false;
		target_attacked = false;
		last_time = (clock_t)0;

		//periodCount = 0;

		e = std::default_random_engine(std::time(0));
		u = std::uniform_real_distribution<float>(0, 1); // 0~1�ľ��ȷֲ�
		deltax = u(e) / 2 - 0.25;
		deltay = u(e) * 1.3333 - 0.66667;
	}

	void init()
	{
	}

	// �Ƿ��Ǻ췽��Ӫ
	bool is_redgrp()
	{
		return self.base.id.grp_id == 0;
	}

	// �ϱ��Էɵ��߼�ʵ�֡�
	// ����������ϣ����ǿ��԰ѵ���̬������Ϊ˫�����������
	// ������ͨ���Էɣ�����ʵ�ֽӽ����״﷢�ֵ�ģ�⡣ û�б��õ�
	void flat_flight(TacSdkSituationUpdate *situation)
	{
		// ����TAC-SDK�ṩ�Ĺ켣׷�ٽӿڣ�ʵ�ֶԷɡ�
		TacSdkTrajPoint p;
		p.h = 5500;
		p.lon = situation->self.base.dof.lon;
		p.ref_phi = situation->self.base.dof.phi;
		p.ref_phi = 0.0;
		if (situation->self.base.dof.psi > 100)
		{
			p.lat = situation->self.base.dof.lat - 0.1;
			p.vel = 250;
		}
		else
		{
			p.lat = situation->self.base.dof.lat + 0.1;
			p.vel = 250;
		}
		tac_entity->TrajPointSet(tac_entity, &p);
		trjpoint_set = true;
	}

	// ������һ��Ŀ���, ���Ƿɻ�����һ������λ����Ҫ�������, û�б��õ�
	void goal_flight(TacSdkSituationUpdate *situation, TacSdk6DOF goal)
	{
		float interval = 0.1;
		float pi = 3.14159265;
		TacSdk6DOF self = situation->self.base.dof;
		TacSdkTrajPoint p;
		p.h = 5500;
		//if (threat_reported) {
		//	std::cout << "��⵼�����룺" << g_threat.missile_dist << std::endl;
		//}

		p.ref_phi = self.phi;
		p.ref_phi = 0.0;

		float dis = sqrt(pow(self.lat - goal.lat, 2) + pow(self.lon - goal.lon, 2));
		float portion = interval / dis;
		float delta_lat = portion * (goal.lat - self.lat); // 0.1 * delta_lat / dis
		float delta_lon = portion * (goal.lon - self.lon); // 0.1 * delta_lon / dis
		//float goal_lat = self.lat + portion * (goal.lat - self.lat);
		//float goal_lon = self.lon + portion * (goal.lon - self.lon);

		float psi = self.psi / 180 * pi;
		float unit_lat = cos(psi);
		float unit_lon = sin(psi);
		float dot = delta_lat * unit_lat + delta_lon * unit_lon;

		if (dot > interval * cos(pi / 3))
		{
			p.lat = self.lat + delta_lat;
			p.lon = self.lon + delta_lon;
		}
		else
		{
			p.lat = self.lat + (delta_lat + unit_lat * interval) / 2;
			p.lon = self.lon + (delta_lon + unit_lon * interval) / 2;
		}
		//float theta = atan2(goal.lat - self.lat, goal.lon - self.lon);
		//p.lat = self.lat + 0.1 * sin(theta);
		//p.lon = self.lon + 0.1 * cos(theta);

		//p.lat = self.lat - 0.1;
		//p.lon = self.lon + 0.1;
		p.vel = 250;
		tac_entity->TrajPointSet(tac_entity, &p);
	}

	// ��ʼ��entity
	Env_Entity *base2entity(TacSdkEntityBaseInfo base)
	{
		Env_Entity_Dof *dof = new Env_Entity_Dof();
		dof->set_lat(base.dof.lat);
		dof->set_lon(base.dof.lon);
		dof->set_height(base.dof.height);
		dof->set_phi(base.dof.phi);
		dof->set_theta(base.dof.theta);
		dof->set_psi(base.dof.psi);

		Env_Entity_Velocity3D *vel = new Env_Entity_Velocity3D();
		vel->set_vel_north(base.vel.vel_north);
		vel->set_vel_east(base.vel.vel_east);
		vel->set_vel_down(base.vel.vel_down);

		Env_Entity *self = new Env_Entity();
		self->set_allocated_dof(dof);
		self->set_allocated_vel(vel);
		self->set_id(base.id.carrier_id);

		return self;
	}

	void reset() {
		target_reported = false;
		threat_reported = false;
	}

	// ����entity�õ�env��Ϣ
	Env *get_state_env(TacSdkSituationUpdate *situation)
	{
		Env *env = new Env();
		Env_Entity *self = base2entity(situation->self.base);
		env->set_allocated_self(self);
		env->set_num_wpn(situation->self.num_wpn);
		env->set_radar_on(true);

		Env_Goal *goal = new Env_Goal();
		goal->set_lat(g_goal_y);
		goal->set_lon(g_goal_x);
		goal->set_height(5500);
		env->set_allocated_goal(goal);

		env->set_detect_enemy(target_reported);
		if (target_reported)
		{
			Env_Entity *enemy = base2entity(situation->target_list.enemies[0]);
			env->set_allocated_enemy(enemy);
		}

		env->set_detect_missle(threat_reported);
		if (threat_reported)
		{
			Env_Missle *missle = new Env_Missle();
			missle->set_dir(g_threat.missile_dir);
			missle->set_dist(g_threat.missile_dist);
			env->set_allocated_missle(missle);
		}

		env->set_red_crash(red_crash);
		env->set_blue_crash(blue_crash);

		reset();

		return env;
	}

	// ��envת��state����model
	void send_state(TacSdkSituationUpdate *situation)
	{
		Env *env = get_state_env(situation);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	// ��envת��state����model
	void send_step(TacSdkSituationUpdate *situation)
	{
		Env *env = get_state_env(situation);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	// ����model��������action��ִ��, Ȼ���state���ظ�model
	void step(TacSdkSituationUpdate *situation, uint64_t tick)
	{
		// ����TAC_SDK�����̬���еı�������
		static int cnt = 0; // ���
		static int blue_dir_sign = -1;
		std::ofstream out("E:\\Window10\\E2\\workspace\\Project\\skln_working\\out.txt", std::ios::app);
		self = situation->self;

		// �����״�
		turn_on_radar(tick);

		//periodCount++;
		if (is_redgrp())
		{
			/*�췽 - ������*/
			if (cnt > 0)
			{
				cnt--;
				return;
			}

			Action action;
			zmq::message_t request;

			//out << "red period:" << periodCount << std::endl;
			if (FIRST)
			{
				context = zmq::context_t{1};
				socket = zmq::socket_t{context, zmq::socket_type::rep};
				items[0] = {socket, 0, ZMQ_POLLIN, 0};
				socket.bind("tcp://*:5555");

				socket.recv(request, zmq::recv_flags::none); // ����reset
				action.ParseFromString(request.to_string());

				send_state(situation);

				if (self.base.dof.lon < BlueX - 0.5)
				{ // ���
					std::cout << "���" << std::endl;
					g_goal_x = BlueX + fabs(deltax);
					g_goal_y = BlueY + deltay;
				}
				else if (self.base.dof.lon > BlueX + 0.5)
				{ // �Ҳ�
					std::cout << "�Ҳ�" << std::endl;
					g_goal_x = BlueX - fabs(deltax);
					g_goal_y = BlueY + deltay;
				}
				else if (self.base.dof.lat > BlueY + 0.5)
				{ // �Ϸ�
					std::cout << "�Ϸ�" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY - fabs(deltay);
				}
				else
				{
					std::cout << "�·�" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY + fabs(deltay);
				}
				std::cout << "delta����" << deltax << ", " << deltay << "��" << std::endl;
				std::cout << "Ŀ��㣺��" << g_goal_x << ", " << g_goal_y << "��" << std::endl;
				FIRST = false;
			}
			else
			{
				send_step(situation);
				out << "red send step done ..." << std::endl;
			}
			socket.recv(request, zmq::recv_flags::none); // ����action
			action.ParseFromString(request.to_string());

			out << "red recv done ..." << std::endl;

			// take action
			Action_TrajPoint point = action.point();
			TacSdkTrajPoint p;
			p.lon = situation->self.base.dof.lon + point.lon();
			p.lat = situation->self.base.dof.lat + point.lat();
			p.ref_phi = situation->self.base.dof.phi;
			p.ref_phi = 0.0;
			p.h = 5500;
			p.vel = 250;

			tac_entity->TrajPointSet(tac_entity, &p);

			//if (action.deploy() && self.num_wpn > 0) {
			//	tac_entity->Deploy(tac_entity,
			//		self.wpn_list[0].wpn_id,
			//		situation->target_list.enemies[0].id.carrier_id,
			//		NULL);
			//}
			cnt = LOOPTIME;
			//flat_flight(situation);

			/*Ŀ�궨�����*/
			//float pi = 3.14159265;
			//static float last_threat_dis = 10000;
			//TacSdk6DOF goal;
			//goal.height = 5500;
			//goal.lat = g_goal_y;
			//goal.lon = g_goal_x;

			//if (threat_reported && g_threat.missile_dist < last_threat_dis) {
			//	float theta = g_threat.missile_dir;
			//	std::cout << "��⵼�����룺" << g_threat.missile_dist << std::endl;
			//	std::cout << "��������" << theta << std::endl;
			//	std::cout << "�ɻ�����" << self.base.dof.psi << std::endl;
			//	if (theta > self.base.dof.psi) {
			//		theta -= pi / 2;
			//	}
			//	else {
			//		theta += pi / 2;
			//	}
			//	goal.lat = self.base.dof.lat + 0.1 * sin(theta);
			//	goal.lon = self.base.dof.lon + 0.1 * cos(theta);
			//	std::cout << "Ŀ�ľ��ȣ�" << goal.lon << std::endl;
			//	std::cout << "Ŀ��γ�ȣ�" << goal.lat << std::endl;
			//	last_threat_dis = g_threat.missile_dist;
			//}
			//float blue_x, blue_y;
			//float dis_blue_red = -1;
			//float red_y = self.base.dof.lat;
			//float red_x = self.base.dof.lon;
			//float missle_dis = 0.3; // �������䷶Χ
			//if (situation->target_list.num_enemies > 0) {
			//	blue_y = situation->target_list.enemies[0].dof.lat;
			//	blue_x = situation->target_list.enemies[0].dof.lon;
			//	dis_blue_red = sqrt(pow(blue_y - red_y, 2) + pow(blue_x - red_x, 2));
			//	std::cout << "�������룺" << dis_blue_red << std::endl;

			//	if (dis_blue_red <= missle_dis && !target_attacked && self.num_wpn > 0) {
			//		std::cout << "���䵼��" << std::endl;
			//		tac_entity->Deploy(tac_entity,
			//			self.wpn_list[0].wpn_id,
			//			situation->target_list.enemies[0].id.carrier_id,
			//			NULL);
			//		target_attacked = true;
			//	}
			//}
			//else {
			//	dis_blue_red = -1;
			//}
			//goal_flight(situation, goal);
		}
		else
		{
			/*���� - ���ط�*/
			// ����TAC-SDK�ṩ�Ĺ켣׷�ٽӿڣ�ʵ�ֶԷɡ�
			// ����Χ�Ʒ������򻷷ɣ�ע�⾭γ����ͳ��ȵĵ�λ �� red�췽��γ����
			if (FIRST)
			{
				if (self.base.dof.psi < 45)
				{
					std::cout << "�·�" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY + fabs(deltay);
					if (deltax < 0)
						blue_dir_sign = 1;
				}
				else if (self.base.dof.psi < 135)
				{
					std::cout << "���" << std::endl;
					g_goal_x = BlueX + fabs(deltax);
					g_goal_y = BlueY + deltay;
					if (deltay > 0)
						blue_dir_sign = 1;
				}
				else if (self.base.dof.psi < 225)
				{
					std::cout << "�Ϸ�" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY - fabs(deltay);
					if (deltax > 0)
						blue_dir_sign = 1;
				}
				else
				{
					std::cout << "�Ҳ�" << std::endl;
					g_goal_x = BlueX - fabs(deltax);
					g_goal_y = BlueY + deltay;
					if (deltay < 0)
						blue_dir_sign = 1;
				}
				std::cout << "delta����" << deltax << ", " << deltay << "��" << std::endl;
				std::cout << "Ŀ��㣺��" << g_goal_x << ", " << g_goal_y << "��" << std::endl;
				std::cout << "���ţ�" << blue_dir_sign << std::endl;
				FIRST = false;
				return;
			}
			// ������û�õ�
			float goal_R = 0.4;		// �������׷���뾶 R = 0.04
			float blue_r = 0.7;		// �������ظ�֪��Χ
			float missle_dis = 0.3; // �������䷶Χ

			TacSdkTrajPoint p;
			p.ref_phi = self.base.dof.phi;
			p.ref_phi = 0.0;
			p.h = 5500;
			p.vel = 240;

			// �������꣬�췽����
			float blue_y = self.base.dof.lat;
			float blue_x = self.base.dof.lon;
			float red_y, red_x;
			float dis_blue_red = -1;

			//std::cout << "���� delta����" << deltax << ", " << deltay << "��" << std::endl;
			//std::cout << "���� Ŀ��㣺��" << g_goal_x << ", " << g_goal_y << "��" << std::endl;
			//std::cout << "�������꣺(" << blue_x << ", " << blue_y << ")" << std::endl;
			//std::cout << "�����Ƕȣ�" << self.base.dof.phi << ", " << self.base.dof.psi << ", " << self.base.dof.theta << std::endl;

			float dis_blue_goal = sqrt(pow(blue_y - g_goal_y, 2) + pow(blue_x - g_goal_x, 2));
			std::cout << "������Ŀ�����룺" << dis_blue_goal << std::endl;
			//dis_br = sqrt(pow(blue_lat - red_lat, 2) + pow(blue_lon - red_lon, 2)); // red_lat �� red_lon

			//out << "blue period: " << periodCount << std::endl;

			if (blue_crash)
			{
				out << "blue crash down !!!" << std::endl;
			}

			//if (situation->target_list.num_enemies > 0) {
			//	red_y = situation->target_list.enemies[0].dof.lat;
			//	red_x = situation->target_list.enemies[0].dof.lon;
			//	dis_blue_red = sqrt(pow(blue_y - red_y, 2) + pow(blue_x - red_x, 2));
			//	//std::cout << "̽�⵽�췽 ���룺" << dis_blue_red << std::endl;
			//	//std::cout << "��Բ�ľ��룺" << dis_blue_goal << std::endl;
			//}
			//else {
			//	dis_blue_red = -1;
			//}

			//if (dis_blue_red >= 0 && dis_blue_red <= blue_r && dis_blue_goal <= goal_R) {
			//	// ����׷���췽
			//	//std::cout << "׷���췽" << std::endl;
			//	goal_flight(situation, situation->target_list.enemies[0].dof);
			//	if (dis_blue_red <= missle_dis && !target_attacked && self.num_wpn > 0){
			//		//std::cout << "���䵼��" << std::endl;
			//		tac_entity->Deploy(tac_entity,
			//			self.wpn_list[0].wpn_id,
			//			situation->target_list.enemies[0].id.carrier_id,
			//			NULL);
			//		target_attacked = true;
			//	}
			//}
			//else if (dis_blue_goal > goal_R) {
			//	TacSdk6DOF goal;
			//	goal.height = 5500;
			//	goal.lat = g_goal_y;
			//	goal.lon = g_goal_x;
			//	std::cout << "������һĿ��㣺��" << goal.lon << ", " << goal.lat << "��" << std::endl;
			//	goal_flight(situation, goal);
			//}
			//else {
			// ����
			//g_goal_x = -121.518;
			//g_goal_y = 36.4035;
			//blue_dir_sign = 1;

			//float theta = atan2(blue_y - g_goal_y, blue_x - g_goal_x);
			////std::cout << "theta:" << theta << std::endl;

			//theta += blue_dir_sign * acos(-1) / 12;

			//p.lat = g_goal_y + GoalRadius * sin(theta);
			//p.lon = g_goal_x + GoalRadius * cos(theta);

			////std::cout << "���� ������һĿ��㣺��" << p.lon << ", " << p.lat << "��" << std::endl;
			////tac_entity->TrajPointSet(tac_entity, &p);
			//TacSdk6DOF goal;
			//goal.height = 5500;
			//goal.lat = p.lat;
			//goal.lon = p.lon;
			//goal_flight(situation, goal);
			//out << "blue goal flight ... " << std::endl;

			//}
			//flat_flight(situation);
		}
		// ƽ��
		//flat_flight(situation);

		// attack
		//attack(situation);
	};

	// 1v1����, Ŀǰû�б��õ�
	void attack(TacSdkSituationUpdate *situation)
	{
		if (!is_redgrp())
		{
			//ģ��췽��������
			return;
		}
		if (target_attacked)
		{
			return;
		}

		if (situation->target_list.num_enemies == 0)
		{
			return;
		}
		if (situation->self.num_wpn == 0)
		{
			return;
		}
		printf("enemies pos:%lf\n", situation->enemies[0].base.dof.lat);
		//ģ���ж��Ƿ������Ч�������롣
		double delta_lat = abs(self.base.dof.lat - situation->enemies[0].base.dof.lat);
		double delta_lon = abs(self.base.dof.lon - situation->enemies[0].base.dof.lon);
		double dist = sqrt(pow(delta_lat, 2) + pow(delta_lon, 2));
		if (dist < 0.3)
		{
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
	void notify_combat_finished()
	{
		if (is_redgrp()) {
			red_crash = true;
		}
		else {
			blue_crash = true;
		}
	}

	// ���״�
	void turn_on_radar(uint64_t tick)
	{
		if (radar_on == true)
			return;
		//if (tick < 300) {
		//	return;
		//}
		uint8_t on = 1;
		tac_entity->SensorControl(tac_entity, 0, TACSDK_SENSOR_CMD_ONOFF, (void *)&on, sizeof(uint8_t));
		radar_on = true;
	}

	// ���ֵл���ѡ������Ŀ��
	void target_found(TacSdkTargetList *tgt)
	{
		// ѡ��Ŀ��͹�������
		for (int i = 0; i < tgt->num_enemies; i++)
		{
			uint16_t enemyid = tgt->enemies[i].id.carrier_id;
			tac_entity->TargetSelect(tac_entity, true, enemyid,
									 self.wpn_list[0].wpn_id);
		}
		target_reported = true;
	}

	// ������в�Ĵ���
	void threat_detected(TacThreatingMissile *missile)
	{
		g_threat.missile_dist = missile->missile_dist;
		g_threat.missile_dir = missile->missile_dir;
		g_threat.missile_id = missile->missile_id;
		threat_reported = true;
		//std::cout << "��⵼������:" << missile->missile_dist << std::endl;
		//std::cout << "��⵼���Ƕ�:" << missile->missile_dir << std::endl;
	}
};

// Ϊ�˶Խ�ƽ̨, Ҫ���ݵ�ʵ��ָ��.
typedef struct
{
	TacSdkAi tak_sdk;
	MyAiModelWrapper ai_wrapper;
} AiModelInstance;