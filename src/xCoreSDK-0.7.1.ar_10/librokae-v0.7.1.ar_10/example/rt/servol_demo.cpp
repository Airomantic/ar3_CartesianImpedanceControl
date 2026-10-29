/**
 * @file servo_l_demo.cpp
 * @brief 实时模式 - ServoL功能demo (笛卡尔空间伺服)
 * 本示例演示ServoL实现空间矩形轨迹运动
 * 
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <thread>
#include <ctime>
#include <cerrno>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

// 用户指令下发周期(s)
constexpr double planPeriod = 0.02; 

// 延迟发送函数
void busy_wait(int milliseconds) {
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::milliseconds(milliseconds);
    while (std::chrono::high_resolution_clock::now() < end) {
    }
}

int main() {
    using namespace std;
    rokae::ArRobot robot;
    std::error_code ec;
    try {
        robot.connectToRobot("192.168.2.160", "192.168.2.100");//机器人ip、上位机ip
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 0;
    }
    
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    // 必做：启用实时模式
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    try {
        auto rtCon = robot.getRtMotionController().lock();
        auto model = robot.model();
        
        // 设置要接收数据（接收测量的关节位置和笛卡尔位姿）
        robot.startReceiveRobotState(std::chrono::milliseconds(1), 
                                     {RtSupportedFields::jointPos_m, 
                                      RtSupportedFields::tcpPose_m});

        std::array<double, 7> jntPos{};
        std::array<double, 7> q_drag_xm3 = {0, M_PI/6, 0,  M_PI/3, 0,  0};
        std::array<double, 7> q2_drag_xm3 ={0, 0,  0, 0, 0, 0};

        while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
        
        jntPos = robot.jointPos(ec);
        // 运行至初始位置
        rtCon->MoveJ(0.3, robot.jointPos(ec), q_drag_xm3);
        
        // 必做：启用ServoL功能（笛卡尔空间伺服）
        // 周期下发不稳定时，可使用前瞻时间和增益调节，默认前瞻时间为一个指令周期，增益为0
        rtCon->setServoJoint(planPeriod, planPeriod, 0, ec);
        rtCon->startMove(RtControllerMode::cartesianPosition);
        // 获取初始笛卡尔位姿
        CartesianPosition initialPose;
        Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), initialPose.pos);
        
        // 提取初始位置和姿态
        Eigen::Matrix3d rot_initial;
        Eigen::Vector3d trans_initial;
        Utils::arrayToTransMatrix(initialPose.pos, rot_initial, trans_initial);
        
        std::cout << "初始位姿: X=" << trans_initial[0] 
                  << ", Y=" << trans_initial[1] 
                  << ", Z=" << trans_initial[2] << std::endl;

        // ── 预计算矩形轨迹（RT 循环外，不占用实时时间）──────────────────────
        constexpr double rect_x    = 0.05;        // X 方向尺寸 50mm
        constexpr double rect_y    = 0.05;        // Y 方向尺寸 50mm
        constexpr double edge_time = 4.0;         // 每条边运动时间 4s
        constexpr double total_time = edge_time * 4;
        const std::size_t total_frames =
            static_cast<std::size_t>(std::ceil(total_time / planPeriod));

        std::vector<std::array<double, 16>> trajectory;
        trajectory.reserve(total_frames);

        for (std::size_t fi = 0; fi < total_frames; ++fi) {
            double t = fi * planPeriod;
            double x_offset = 0.0, y_offset = 0.0;

            if (t < edge_time) {
                // 边1: 原点 → 右边 (X 增加)
                x_offset = (t / edge_time) * rect_x;
                y_offset = 0.0;
            } else if (t < edge_time * 2) {
                // 边2: 右边 → 右上 (Y 增加)
                x_offset = rect_x;
                y_offset = ((t - edge_time) / edge_time) * rect_y;
            } else if (t < edge_time * 3) {
                // 边3: 右上 → 左上 (X 减少)
                x_offset = rect_x * (1.0 - (t - edge_time * 2) / edge_time);
                y_offset = rect_y;
            } else {
                // 边4: 左上 → 原点 (Y 减少)
                x_offset = 0.0;
                y_offset = rect_y * (1.0 - (t - edge_time * 3) / edge_time);
            }

            Eigen::Vector3d trans_target = trans_initial;
            trans_target[0] += x_offset;
            trans_target[1] += y_offset;

            std::array<double, 16> pos{};
            Utils::transMatrixToArray(rot_initial, trans_target, pos);
            trajectory.push_back(pos);
        }
        std::cout << "矩形轨迹预计算完成，共 " << trajectory.size() << " 帧" << std::endl;

   
        std::size_t idx = 0;
        CartesianPosition cmd;  
        std::cout << "矩形轨迹开始循环运行，按 Ctrl+C 终止..." << std::endl;
        while (true) {
            robot.updateRobotState(std::chrono::milliseconds(1));
            cmd.pos = trajectory[idx];
            rtCon->sendCommand(cmd);
            busy_wait(static_cast<int>(planPeriod * 1000));
            ++idx;
            if (idx >= trajectory.size()) {
                idx = 0;  // 回到起点，继续下一圈
            }
        }
        
        // 发送结束指令
        CartesianPosition final_cmd;
        Utils::transMatrixToArray(rot_initial, trans_initial, final_cmd.pos);
        final_cmd.setFinished();
        rtCon->sendCommand(final_cmd);
        //必做：关闭servol功能
        rtCon->stopServoJoint();

    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
    rtCon->MoveJ(0.3, robot.jointPos(ec), q2_drag_xm3);
    std::cout << "ServoL控制结束" << std::endl;

    // 关闭实时模式
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
