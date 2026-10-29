/**
 * @file mass_matrix_jacobian_dot_demo.cpp
 * @brief 实时模式 - 动力学计算测试 (7轴机器人)
 * 此示例需要使用xMateModel模型库，请设置编译选项XCORE_USE_XMATE_MODEL=ON
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "../function_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;

  // 创建机器人对象 (7轴AR系列)
  rokae::ArRobot robot;
  std::error_code ec;

  // 连接到机器人
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100"); // 本机地址192.168.0.100
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
    return 0;
  }
  auto model =robot.model();

  // 7轴机器人初始位置
  std::array<double, 7> q_init = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

  // 切换到实时模式
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  try {
    auto rtCon = robot.getRtMotionController().lock();

    // 启动接收机器人状态数据（位置、速度、加速度）
    robot.startReceiveRobotState(std::chrono::milliseconds(1), 
                                 {RtSupportedFields::jointPos_m, 
                                  RtSupportedFields::jointVel_m, 
                                  RtSupportedFields::jointAcc_m});

    // 设置滤波
    rtCon->setFilterLimit(true, 10);
    rtCon->setFilterFrequency(10, 10, 10, ec);

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_init);

    double time = 0;
    bool has_printed = false; // 标记是否已打印

    std::array<double, 7> jntPos{};

    // 定义回调函数
    std::function<JointPosition()> callback = [&, rtCon](){
      time += 0.001;
      
      // 更新机器人状态数据
      robot.updateRobotState(std::chrono::milliseconds(1));
      
      // 余弦插值型 S 曲线示例
      double delta_angle = M_PI / 50.0 * (1 - std::cos(M_PI / 2.5 * time));
      JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                            jntPos[2] - delta_angle, jntPos[3] + delta_angle, 
                            jntPos[4] - delta_angle, jntPos[5] + delta_angle,
                            jntPos[6] + delta_angle}};

      // 在 time >= 10 秒时打印一次动力学信息
      if(time >= 10.0 && !has_printed) {
        has_printed = true;
        
        try {
          // 获取当前机器人状态数据
          std::array<double, 7> measured_pos{};
          std::array<double, 7> measured_vel{};
          std::array<double, 7> measured_acc{};
          robot.getStateData(RtSupportedFields::jointPos_m, measured_pos);
          robot.getStateData(RtSupportedFields::jointVel_m, measured_vel);
          robot.getStateData(RtSupportedFields::jointAcc_m, measured_acc);

          std::cout << "\n========== 动力学计算结果 (t = 10.0s) ==========" << std::endl;
          
          // 关节位置
          std::cout << "关节位置 (rad):        ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << measured_pos[i] << " ";
          }
          std::cout << std::endl;
          
          // 关节速度
          std::cout << "关节速度 (rad/s):      ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << measured_vel[i] << " ";
          }
          std::cout << std::endl;
          
          // 关节加速度
          std::cout << "关节加速度 (rad/s²):   ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << measured_acc[i] << " ";
          }
          std::cout << std::endl;
          
          // 质量矩阵对角线
          auto massMatrix = model.getMassMatrix(measured_pos);
          std::cout << "质量矩阵对角线 (kg·m²): ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << massMatrix[i*7 + i] << " ";
          }
          std::cout << std::endl;

          // 科里奥利力（新接口）
          auto coriolisVector = model.getCoriolisVector(measured_pos, measured_vel);
          std::cout << "科里奥利力-新接口 (Nm): ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << coriolisVector[i] << " ";
          }
          std::cout << std::endl;
          
          // 科里奥利力（旧接口）
          std::array<double, 7> trq_full, trq_inertia, trq_coriolis, trq_gravity;
          model.getTorqueNoFriction(measured_pos, measured_vel, measured_acc, 
                                     trq_full, trq_inertia, trq_coriolis, trq_gravity);
          std::cout << "科里奥利力-旧接口 (Nm): ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << trq_coriolis[i] << " ";
          }
          std::cout << std::endl;

          // 雅可比矩阵导数第一行
          auto jacobianDot = model.jacobianDot(measured_pos, measured_vel);
          std::cout << "雅可比导数第1行 (1/s):  ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << jacobianDot[i] << " ";
          }
          std::cout << std::endl;

          // ========== 正装模式下的动力学计算 ==========
          std::cout << "\n---------- 正装模式 (A=0, B=0, C=0) ----------" << std::endl;
          model.setGravityDirection(0, 0, -9.81);  // 正装
          std::array<double, 7> full_normal{}, inertia_normal{}, coriolis_normal{}, gravity_normal{};
          model.getTorqueNoFriction(measured_pos, measured_vel, measured_acc, 
                                    full_normal, inertia_normal, coriolis_normal, gravity_normal);
          std::cout << "重力矩 (Nm):            ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << gravity_normal[i] << " ";
          }
          std::cout << std::endl;

          // ========== 侧装模式下的动力学计算 (A=90度) ==========
          std::cout << "\n---------- 侧装模式 (A=90, B=0, C=0) ----------" << std::endl;
          // A=90度: 基坐标系绕世界X轴旋转90度
          // 重力矢量: g_base = Rx(-90) * [0, 0, -9.81]^T = [0, -9.81, 0]^T
          model.setGravityDirection(0, -9.81, 0);  // 侧装 A=90
          std::array<double, 7> full_side{}, inertia_side{}, coriolis_side{}, gravity_side{};
          model.getTorqueNoFriction(measured_pos, measured_vel, measured_acc, 
                                    full_side, inertia_side, coriolis_side, gravity_side);
          std::cout << "重力矩 (Nm):            ";
          for(int i = 0; i < 7; i++) {
            std::cout << std::setw(8) << std::setprecision(4) << gravity_side[i] << " ";
          }
          std::cout << std::endl;

          // 恢复为正装模式
          model.setGravityDirection(0, 0, -9.81);

        } catch (const std::exception &e) {
          std::cerr << "动力学计算错误: " << e.what() << std::endl;
        }
      }

      if(time > 15) {  // 15秒后结束
        cmd.setFinished();
      }
      return cmd;
    };

    // 更新机器人状态，确保数据可用
    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
    
    rtCon->setControlLoop(callback);
    jntPos = robot.jointPos(ec);
    
    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);
    rtCon->startLoop(true);

    // 将控制模式设为空闲并下电
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false, ec);

  } catch (const std::exception &e) {
    std::cerr << "错误: " << e.what() << std::endl;
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false, ec);
  }

  return 0;
}
