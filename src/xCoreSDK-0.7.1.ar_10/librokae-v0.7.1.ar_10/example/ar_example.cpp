/**
 * @file ar_example.cpp
 * @brief AR机型示例，点位基于AR5-5_0.7L-W4C1C1,侧装，A=90°
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include "rokae/model.h"

using namespace rokae;
std::ostream &os = std::cout;

void waitRobot(BaseRobot &robot){
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot.operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            running = false;
        }
    }
}

void print_hmi_pose(const CartesianPosition &cart_pose) {
    os << "print_hmi_pose" << std::endl;
    os << "pos: ";
    for (size_t i = 0; i < cart_pose.trans.size(); ++i) {
        double scaled_trans = cart_pose.trans[i] * 1000.0;
        if (i > 0)
            os << ", ";
        os << scaled_trans;
    }
    os << std::endl;

    os << "rpy: ";
    const double RAD_TO_DEG = 180.0 / M_PI;
    for (size_t i = 0; i < cart_pose.rpy.size(); ++i) {
        double scaled_rpy = cart_pose.rpy[i] * RAD_TO_DEG;
        if (i > 0)
            os << ", ";
        os << scaled_rpy;
    }
    os << std::endl;
}

void print_cart_pose(const CartesianPosition &cart_pose) {
    os << "elbow: " << cart_pose.elbow * 180 / M_PI << std::endl;
    os << "has elbow: " << cart_pose.hasElbow << std::endl;
    os << "confdata: " << cart_pose.confData << std::endl;
    if (!cart_pose.external.empty())
        os << "external: " << cart_pose.external << std::endl;
    // os <<"trans: " << cart_pose.trans << std::endl;
    // os << "rpy: " << cart_pose.rpy << std::endl;
    print_hmi_pose(cart_pose);
}

void get_car_pose(ArRobot &robot, error_code &ec) {
    auto pose = robot.posture(CoordinateType::endInRef, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    // os << "pos: " << pose[0] << ", " << pose[1] << ", " << pose[2] << ", " << pose[3]
    // << ", " << pose[4] << ", " << pose[5] << std::endl;

    os << "hmi pos: " << pose[0] * 1000 << ", " << pose[1] * 1000 << ", " << pose[2] * 1000 << ", " << pose[3] * 180 / M_PI
       << ", " << pose[4] * 180 / M_PI << ", " << pose[5] * 180 / M_PI << std::endl;

    auto cart_pose = robot.cartPosture(CoordinateType::endInRef, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    print_cart_pose(cart_pose);
}

void get_joint_pose(ArRobot &robot, error_code &ec) {
    auto pose = robot.jointPos(ec);
    // os << "joint pose: " << pose[0] << ", " << pose[1] << ", " << pose[2] << ", " << pose[3]
    // << ", " << pose[4] << ", " << pose[5]  << "," << pose[6] << std::endl;

    os << "hmi joint pose: " << pose[0] * 180 / M_PI << ", " << pose[1] * 180 / M_PI << ", " << pose[2] * 180 / M_PI << ", " << pose[3] * 180 / M_PI
       << ", " << pose[4] * 180 / M_PI << ", " << pose[5] * 180 / M_PI << ", " << pose[6] * 180 / M_PI << std::endl;
}

void calcFk(ArRobot &robot, error_code &ec) {
    auto start_angle = robot.jointPos(ec);
    auto robot_model = robot.model();
    auto cart_pose = robot_model.calcFk(start_angle, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    print_cart_pose(cart_pose);

    Toolset toolset;
    toolset.end.trans = {0.01, 0.01, 0.01};
    toolset.end.rpy = {0.01, 0.01, 0.01};
    toolset.ref.trans = {0, 0, 0};
    toolset.ref.rpy = {0, 0, 0};
    cart_pose = robot_model.calcFk(start_angle, toolset, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    print_cart_pose(cart_pose);
}

void calcIk(ArRobot &robot, error_code &ec) {
    CartesianPosition cart_pos({0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708});
    cart_pos.elbow = 16.6993 * M_PI / 180;
    auto robot_model = robot.model();

    auto joint_pos = robot_model.calcIk(cart_pos, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    // os << "joint_pos: " << joint_pos[0] << ", " << joint_pos[1] << ", " << joint_pos[2] << ", " << joint_pos[3]
    // << ", " << joint_pos[4] << ", " << joint_pos[5] << ", " << joint_pos[6] << std::endl;
    os << "hmi joint_pos: " << joint_pos[0] * 180 / M_PI << ", " << joint_pos[1] * 180 / M_PI << ", " << joint_pos[2] * 180 / M_PI << ", " << joint_pos[3] * 180 / M_PI
       << ", " << joint_pos[4] * 180 / M_PI << ", " << joint_pos[5] * 180 / M_PI << ", " << joint_pos[6] * 180 / M_PI << std::endl;

    Toolset toolset;
    toolset.end.trans = {0.01, 0.01, 0.01};
    toolset.end.rpy = {0.01, 0.01, 0.01};
    toolset.ref.trans = {0, 0, 0};
    toolset.ref.rpy = {0, 0, 0};
    CartesianPosition cart_pos2({0.54466, -0.441432, 0.01, 1.5807, -0.0099995, 1.5808});
    cart_pos2.elbow = 16.6993 * M_PI / 180;
    joint_pos = robot_model.calcIk(cart_pos2, toolset, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    // os << "joint_pos: " << joint_pos[0] << ", " << joint_pos[1] << ", " << joint_pos[2] << ", " << joint_pos[3]
    // << ", " << joint_pos[4] << ", " << joint_pos[5] << ", " << joint_pos[6] << std::endl;
    os << "hmi joint_pos: " << joint_pos[0] * 180 / M_PI << ", " << joint_pos[1] * 180 / M_PI << ", " << joint_pos[2] * 180 / M_PI << ", " << joint_pos[3] * 180 / M_PI
       << ", " << joint_pos[4] * 180 / M_PI << ", " << joint_pos[5] * 180 / M_PI << ", " << joint_pos[6] * 180 / M_PI << std::endl;
}

void movej(ArRobot &robot, error_code &ec) {
    robot.setDefaultConfOpt(false, ec);
    CartesianPosition cart_pos1({0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708});
    cart_pos1.elbow = 16.6993 * M_PI / 180;
    CartesianPosition cart_pos2({0.53466, -0.351432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708});
    cart_pos2.elbow = 16.6993 * M_PI / 180;
    MoveJCommand movejcmd(cart_pos1, 100, 10);
    MoveJCommand movejcmd2(cart_pos2, 100, 10);
    std::string cmdID = "";
    robot.moveAppend(
        {movejcmd, movejcmd2}, cmdID, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    robot.moveStart(ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    waitRobot(robot);
}

void movel(ArRobot &robot, error_code &ec) {
    CartesianPosition cart_pos1({0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708});
    cart_pos1.elbow = 16.6993 * M_PI / 180;
    CartesianPosition cart_pos2({0.53466, -0.351432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708});
    cart_pos2.elbow = 16.6993 * M_PI / 180;
    MoveLCommand movejcmd(cart_pos1, 100, 10);
    MoveLCommand movejcmd2(cart_pos2, 100, 10);
    std::string cmdID = "";
    robot.moveAppend({movejcmd, movejcmd2}, cmdID, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    robot.moveStart(ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    waitRobot(robot);
}

void moveabsj(ArRobot &robot, error_code &ec) {
    MoveAbsJCommand absjcmd1({0.0, 0.667174589250972, 0.0, 0.4152523686488143, 0.0, 0.48836936889511035, 0.0}, 100, 10);
    MoveAbsJCommand absjcmd2({0.0, 0.5235987755982988, 0.0, 1.0471975511965976, 0.0, 0, 0.0}, 100, 10);
    std::string cmdID = "";
    robot.moveAppend({absjcmd1, absjcmd2}, cmdID, ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    robot.moveStart(ec);
    if (ec)
        os << "ec: " << ec.value() << std::endl;
    waitRobot(robot);
}

void rt_cart_controller(ArRobot &robot, error_code &ec) {
    std::array<double, 6> start_pos{0.53466, -0.351432, 0, M_PI / 2, 0, M_PI / 2};
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    CartesianPosition start_tcp_point(start_pos);
    MoveJCommand move_j_cmd(start_tcp_point, 100, 10);
    std::string cmdID = "";
    robot.moveAppend({move_j_cmd}, cmdID, ec);
    robot.moveStart(ec);
    waitRobot(robot);
    try {
        robot.setRtNetworkTolerance(100, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        auto rtCon = robot.getRtMotionController().lock();
        rtCon->setFilterLimit(true, 10);
        rtCon->setFilterFrequency(10, 10, 10, ec);
        double time = 0;
        std::array<double, 6> start_pos_rt{0.53466, 0, 0.351432, -M_PI, M_PI / 2, M_PI};
        CartesianPosition cmd;
        cmd.hasElbow = true;
        Utils::postureToTransArray(start_pos_rt, cmd.pos);
        std::function<CartesianPosition()> callback = [&, rtCon]() {
            time += 0.001;
            cmd.pos[11] -= 0.0001;
            if (time > 2) {
                cmd.setFinished();
            }
            return cmd;
        };
        rtCon->setControlLoop(callback);
        rtCon->startMove(RtControllerMode::cartesianPosition);
        rtCon->startLoop(true);
        print(std::cout, "control end");
        robot.setMotionControlMode(MotionControlMode::Idle, ec);
        robot.setPowerState(false, ec);
    }
    catch (const std::exception &e) {
        print(std::cerr, e.what());
        robot.setMotionControlMode(MotionControlMode::Idle, ec);
        robot.setPowerState(false, ec);
    }
}

void rt_joint_controller(ArRobot &robot, error_code &ec) {
    std::array<double, 7> jntPos{0.0, 0.5235987755982988, 0.0, 1.0471975511965976, 0.0, 0.0, 0.0};
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    MoveAbsJCommand moveabsj_cmd({0.0, 0.5235987755982988, 0.0, 1.0471975511965976, 0.0, 0.0, 0.0}, 100, 10);
    std::string cmdID = "";
    robot.moveAppend({moveabsj_cmd}, cmdID, ec);
    robot.moveStart(ec);
    waitRobot(robot);
    try {
        robot.setRtNetworkTolerance(100, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        auto rtCon = robot.getRtMotionController().lock();
        rtCon->setFilterLimit(true, 10);
        double time = 0;
        std::function<JointPosition()> callback = [&, rtCon]() {
            time += 0.001;
            double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time)) / 5;
            JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                                  jntPos[2] - delta_angle,
                                  jntPos[3] + delta_angle, jntPos[4] - delta_angle,
                                  jntPos[5] + delta_angle, jntPos[6] - delta_angle}};

            if (time > 2) {
                cmd.setFinished();
            }
            return cmd;
        };
        rtCon->setControlLoop(callback);
        rtCon->startMove(RtControllerMode::jointPosition);
        rtCon->startLoop(true);
        print(std::cout, "control end");
        robot.setMotionControlMode(MotionControlMode::Idle, ec);
        robot.setPowerState(false, ec);
    }
    catch (const std::exception &e) {
        print(std::cerr, e.what());
        robot.setMotionControlMode(MotionControlMode::Idle, ec);
        robot.setPowerState(false, ec);
    }
}

int main() {
    try
    {
        ArRobot robot("192.168.0.160", "192.168.0.2");
        error_code ec;
        // get_car_pose(robot, ec);
        // get_joint_pose(robot, ec);
        // calcFk(robot, ec);
        // calcIk(robot, ec);
        // movej(robot, ec);
        // movel(robot, ec);
        // moveabsj(robot, ec);
        // rt_cart_controller(robot, ec);
        // rt_joint_controller(robot, ec);
    }
    catch (const std::exception &e) {
        os << e.what() << std::endl;
    }
    return 0;
}