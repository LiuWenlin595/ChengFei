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
	这两个类，是如何组织AI模型与TAC-SDK接口的参考设计。
	采用这个设计，可以把AI模型的业务逻辑与TAC-SDK的接口有效解耦。
*/

// 实际的AI模型的类
// AI模型的实际业务逻辑实现。
// Demo不实现实际的AI算法，逻辑直接在MyAiModelWrapper中处理。
class MyAiModel
{
public:
	void init(){};
};

// 为了对接平台, 对实际的AI模型类进行包装.
// 也可以直接修改原有AI模型,增加TacSdkEntity数属性.
class MyAiModelWrapper : public MyAiModel
{
public:
	TacSdkEntity *tac_entity; // TAC-SDK接口对象
	TacSdkFriend self;		  // 本机数据
	bool radar_on;			  // 雷达是否打开
	bool trjpoint_set;
	bool threat_reported; // 是否发现导弹
	bool target_reported; // 是否发现敌机
	bool target_attacked; // 是否发射出导弹

	TacThreatingMissile g_threat; // 导弹信息
	bool red_crash;				// 红方是否被击中
	bool blue_crash;			// 蓝方是否被击中
	//int periodCount;	// 输出文件判断是否C++在运行

	bool FIRST; // 是否是开局第一帧
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
	float deltax, deltay, g_goal_x, g_goal_y; // x目标点经度, y目标点纬度

	float GoalRadius = 0.1; // 防守环飞半径 r = 0.1

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
		u = std::uniform_real_distribution<float>(0, 1); // 0~1的均匀分布
		deltax = u(e) / 2 - 0.25;
		deltay = u(e) * 1.3333 - 0.66667;
	}

	void init()
	{
	}

	// 是否是红方阵营
	bool is_redgrp()
	{
		return self.base.id.grp_id == 0;
	}

	// 南北对飞的逻辑实现。
	// 在任务设计上，我们可以把敌我态势设置为双方经度相近。
	// 这样，通过对飞，可以实现接近、雷达发现的模拟。 没有被用到
	void flat_flight(TacSdkSituationUpdate *situation)
	{
		// 调用TAC-SDK提供的轨迹追踪接口，实现对飞。
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

	// 给定下一个目标点, 但是飞机的下一个飞行位置需要额外计算, 没有被用到
	void goal_flight(TacSdkSituationUpdate *situation, TacSdk6DOF goal)
	{
		float interval = 0.1;
		float pi = 3.14159265;
		TacSdk6DOF self = situation->self.base.dof;
		TacSdkTrajPoint p;
		p.h = 5500;
		//if (threat_reported) {
		//	std::cout << "检测导弹距离：" << g_threat.missile_dist << std::endl;
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

	// 初始化entity
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

	// 根据entity得到env信息
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

	// 把env转成state传给model
	void send_state(TacSdkSituationUpdate *situation)
	{
		Env *env = get_state_env(situation);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	// 把env转成state传给model
	void send_step(TacSdkSituationUpdate *situation)
	{
		Env *env = get_state_env(situation);

		string env_msg;
		env->SerializeToString(&env_msg);
		socket.send(zmq::buffer(env_msg), zmq::send_flags::none);
	}

	// 接收model传过来的action并执行, 然后把state传回给model
	void step(TacSdkSituationUpdate *situation, uint64_t tick)
	{
		// 保存TAC_SDK传入的态势中的本机数据
		static int cnt = 0; // 间隔
		static int blue_dir_sign = -1;
		std::ofstream out("E:\\Window10\\E2\\workspace\\Project\\skln_working\\out.txt", std::ios::app);
		self = situation->self;

		// 开启雷达
		turn_on_radar(tick);

		//periodCount++;
		if (is_redgrp())
		{
			/*红方 - 攻击方*/
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

				socket.recv(request, zmq::recv_flags::none); // 接收reset
				action.ParseFromString(request.to_string());

				send_state(situation);

				if (self.base.dof.lon < BlueX - 0.5)
				{ // 左侧
					std::cout << "左侧" << std::endl;
					g_goal_x = BlueX + fabs(deltax);
					g_goal_y = BlueY + deltay;
				}
				else if (self.base.dof.lon > BlueX + 0.5)
				{ // 右侧
					std::cout << "右侧" << std::endl;
					g_goal_x = BlueX - fabs(deltax);
					g_goal_y = BlueY + deltay;
				}
				else if (self.base.dof.lat > BlueY + 0.5)
				{ // 上方
					std::cout << "上方" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY - fabs(deltay);
				}
				else
				{
					std::cout << "下方" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY + fabs(deltay);
				}
				std::cout << "delta：（" << deltax << ", " << deltay << "）" << std::endl;
				std::cout << "目标点：（" << g_goal_x << ", " << g_goal_y << "）" << std::endl;
				FIRST = false;
			}
			else
			{
				send_step(situation);
				out << "red send step done ..." << std::endl;
			}
			socket.recv(request, zmq::recv_flags::none); // 接收action
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

			/*目标定点飞行*/
			//float pi = 3.14159265;
			//static float last_threat_dis = 10000;
			//TacSdk6DOF goal;
			//goal.height = 5500;
			//goal.lat = g_goal_y;
			//goal.lon = g_goal_x;

			//if (threat_reported && g_threat.missile_dist < last_threat_dis) {
			//	float theta = g_threat.missile_dir;
			//	std::cout << "检测导弹距离：" << g_threat.missile_dist << std::endl;
			//	std::cout << "导弹方向：" << theta << std::endl;
			//	std::cout << "飞机方向：" << self.base.dof.psi << std::endl;
			//	if (theta > self.base.dof.psi) {
			//		theta -= pi / 2;
			//	}
			//	else {
			//		theta += pi / 2;
			//	}
			//	goal.lat = self.base.dof.lat + 0.1 * sin(theta);
			//	goal.lon = self.base.dof.lon + 0.1 * cos(theta);
			//	std::cout << "目的经度：" << goal.lon << std::endl;
			//	std::cout << "目的纬度：" << goal.lat << std::endl;
			//	last_threat_dis = g_threat.missile_dist;
			//}
			//float blue_x, blue_y;
			//float dis_blue_red = -1;
			//float red_y = self.base.dof.lat;
			//float red_x = self.base.dof.lon;
			//float missle_dis = 0.3; // 导弹发射范围
			//if (situation->target_list.num_enemies > 0) {
			//	blue_y = situation->target_list.enemies[0].dof.lat;
			//	blue_x = situation->target_list.enemies[0].dof.lon;
			//	dis_blue_red = sqrt(pow(blue_y - red_y, 2) + pow(blue_x - red_x, 2));
			//	std::cout << "红蓝距离：" << dis_blue_red << std::endl;

			//	if (dis_blue_red <= missle_dis && !target_attacked && self.num_wpn > 0) {
			//		std::cout << "发射导弹" << std::endl;
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
			/*蓝方 - 防守方*/
			// 调用TAC-SDK提供的轨迹追踪接口，实现对飞。
			// 蓝方围绕防守区域环飞，注意经纬坐标和长度的单位 和 red红方经纬坐标
			if (FIRST)
			{
				if (self.base.dof.psi < 45)
				{
					std::cout << "下方" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY + fabs(deltay);
					if (deltax < 0)
						blue_dir_sign = 1;
				}
				else if (self.base.dof.psi < 135)
				{
					std::cout << "左侧" << std::endl;
					g_goal_x = BlueX + fabs(deltax);
					g_goal_y = BlueY + deltay;
					if (deltay > 0)
						blue_dir_sign = 1;
				}
				else if (self.base.dof.psi < 225)
				{
					std::cout << "上方" << std::endl;
					g_goal_x = BlueX + deltax;
					g_goal_y = BlueY - fabs(deltay);
					if (deltax > 0)
						blue_dir_sign = 1;
				}
				else
				{
					std::cout << "右侧" << std::endl;
					g_goal_x = BlueX - fabs(deltax);
					g_goal_y = BlueY + deltay;
					if (deltay < 0)
						blue_dir_sign = 1;
				}
				std::cout << "delta：（" << deltax << ", " << deltay << "）" << std::endl;
				std::cout << "目标点：（" << g_goal_x << ", " << g_goal_y << "）" << std::endl;
				std::cout << "符号：" << blue_dir_sign << std::endl;
				FIRST = false;
				return;
			}
			// 这三个没用到
			float goal_R = 0.4;		// 防守最大追击半径 R = 0.04
			float blue_r = 0.7;		// 蓝方防守感知范围
			float missle_dis = 0.3; // 导弹发射范围

			TacSdkTrajPoint p;
			p.ref_phi = self.base.dof.phi;
			p.ref_phi = 0.0;
			p.h = 5500;
			p.vel = 240;

			// 蓝方坐标，红方坐标
			float blue_y = self.base.dof.lat;
			float blue_x = self.base.dof.lon;
			float red_y, red_x;
			float dis_blue_red = -1;

			//std::cout << "蓝方 delta：（" << deltax << ", " << deltay << "）" << std::endl;
			//std::cout << "蓝方 目标点：（" << g_goal_x << ", " << g_goal_y << "）" << std::endl;
			//std::cout << "蓝方坐标：(" << blue_x << ", " << blue_y << ")" << std::endl;
			//std::cout << "蓝方角度：" << self.base.dof.phi << ", " << self.base.dof.psi << ", " << self.base.dof.theta << std::endl;

			float dis_blue_goal = sqrt(pow(blue_y - g_goal_y, 2) + pow(blue_x - g_goal_x, 2));
			std::cout << "蓝方与目标点距离：" << dis_blue_goal << std::endl;
			//dis_br = sqrt(pow(blue_lat - red_lat, 2) + pow(blue_lon - red_lon, 2)); // red_lat 和 red_lon

			//out << "blue period: " << periodCount << std::endl;

			if (blue_crash)
			{
				out << "blue crash down !!!" << std::endl;
			}

			//if (situation->target_list.num_enemies > 0) {
			//	red_y = situation->target_list.enemies[0].dof.lat;
			//	red_x = situation->target_list.enemies[0].dof.lon;
			//	dis_blue_red = sqrt(pow(blue_y - red_y, 2) + pow(blue_x - red_x, 2));
			//	//std::cout << "探测到红方 距离：" << dis_blue_red << std::endl;
			//	//std::cout << "与圆心距离：" << dis_blue_goal << std::endl;
			//}
			//else {
			//	dis_blue_red = -1;
			//}

			//if (dis_blue_red >= 0 && dis_blue_red <= blue_r && dis_blue_goal <= goal_R) {
			//	// 蓝方追击红方
			//	//std::cout << "追击红方" << std::endl;
			//	goal_flight(situation, situation->target_list.enemies[0].dof);
			//	if (dis_blue_red <= missle_dis && !target_attacked && self.num_wpn > 0){
			//		//std::cout << "发射导弹" << std::endl;
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
			//	std::cout << "蓝方下一目标点：（" << goal.lon << ", " << goal.lat << "）" << std::endl;
			//	goal_flight(situation, goal);
			//}
			//else {
			// 环绕
			//g_goal_x = -121.518;
			//g_goal_y = 36.4035;
			//blue_dir_sign = 1;

			//float theta = atan2(blue_y - g_goal_y, blue_x - g_goal_x);
			////std::cout << "theta:" << theta << std::endl;

			//theta += blue_dir_sign * acos(-1) / 12;

			//p.lat = g_goal_y + GoalRadius * sin(theta);
			//p.lon = g_goal_x + GoalRadius * cos(theta);

			////std::cout << "环绕 蓝方下一目标点：（" << p.lon << ", " << p.lat << "）" << std::endl;
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
		// 平飞
		//flat_flight(situation);

		// attack
		//attack(situation);
	};

	// 1v1攻击, 目前没有被用到
	void attack(TacSdkSituationUpdate *situation)
	{
		if (!is_redgrp())
		{
			//模拟红方攻击蓝方
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
		//模拟判断是否进入有效攻击距离。
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

	// 演示如何与训练程序通信。
	// AI模型如果要发送数据到训练程序，可以做类似调用。

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

	// 打开雷达
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

	// 发现敌机并选定攻击目标
	void target_found(TacSdkTargetList *tgt)
	{
		// 选定目标和攻击武器
		for (int i = 0; i < tgt->num_enemies; i++)
		{
			uint16_t enemyid = tgt->enemies[i].id.carrier_id;
			tac_entity->TargetSelect(tac_entity, true, enemyid,
									 self.wpn_list[0].wpn_id);
		}
		target_reported = true;
	}

	// 发现威胁的处理
	void threat_detected(TacThreatingMissile *missile)
	{
		g_threat.missile_dist = missile->missile_dist;
		g_threat.missile_dir = missile->missile_dir;
		g_threat.missile_id = missile->missile_id;
		threat_reported = true;
		//std::cout << "检测导弹距离:" << missile->missile_dist << std::endl;
		//std::cout << "检测导弹角度:" << missile->missile_dir << std::endl;
	}
};

// 为了对接平台, 要传递的实例指针.
typedef struct
{
	TacSdkAi tak_sdk;
	MyAiModelWrapper ai_wrapper;
} AiModelInstance;