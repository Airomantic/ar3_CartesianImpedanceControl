/**
 * @file can_linkerbot_control.cpp
 * @brief 末端 CAN 控制灵心巧手（十自由度）五指抓握示例
 *
 * 命令行： [robot_ip] [local_ip] [can_id]
 * 示例：can_linkerbot_control 192.168.71.161 192.168.71.161 0x27
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "rokae/robot.h"

using namespace rokae;

namespace {

void SetupConsoleUtf8() {
#ifdef _WIN32
  SetConsoleOutputCP(65001);
  SetConsoleCP(65001);
#endif
}

void PrintCanFrameRecv(const char *tag, const CANFrame &r, const std::error_code &ec) {
  std::cout << tag << " ec=" << ec.value() << " can_recv: [ " << r.frame_id << ' ' << r.frame_format << ' '
            << r.frame_type << ' ' << r.frame_valid_length;
  for (auto item : r.data) {
    std::cout << ' ' << static_cast<unsigned>(item);
  }
  std::cout << " ]" << std::endl;
}

/**
 * @brief 发一帧 -> 延时 -> 收一帧并打印
 */
void SendSleepRecvOne(xMateErProRobot &robot, CANFrame &frame, std::error_code &ec, int recv_timeout_ms,
                      int sleep_after_send_ms, const char *tag) {
  std::vector<CANFrame> send_frames;
  send_frames.push_back(frame);
  robot.CANSendData("uint8", send_frames, ec);
  std::cout << tag << " CANSendData ec=" << ec.value() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_after_send_ms));
  CANFrame can_recv;
  robot.CANReceiveData(recv_timeout_ms, "uint8", can_recv, ec);
  PrintCanFrameRecv(tag, can_recv, ec);
}

}  // namespace

/**
 * @brief 五指流程：速度 -> 张开 -> 半握 -> 全握 -> 读法向压力 -> 再回到完全张开（每步单独收发）
 */
void RunGraspDemo(xMateErProRobot &robot, int can_id, std::error_code &ec) {
  constexpr int kRecvTimeoutMs = 50;
  constexpr int kSleepAfterSendMs = 1000;

  CANFrame f;
  f.frame_id = can_id;
  f.frame_format = static_cast<int>(CANFormat::STANDARD);
  f.frame_type = static_cast<int>(CANType::CAN);

  // 0x05 / 0x06：关节 1–5 / 6–10 速度（指令码 + 5 字节，共 6 字节有效载荷）
  f.frame_valid_length = 6;
  f.data = {0x05, 0x60, 0x60, 0x60, 0x60, 0x60};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "set_speed_j1_5");

  f.data = {0x06, 0x60, 0x60, 0x60, 0x60, 0x60};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "set_speed_j6_10");

  // 0x01：关节 1–6 位置，DLC 7
  f.frame_valid_length = 7;
  f.data = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "hand_open_j1_6");

  // 0x04：关节 7–10 位置，DLC 5
  f.frame_valid_length = 5;
  f.data = {0x04, 0xFF, 0xFF, 0xFF, 0xFF};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "hand_open_j7_10");

  f.frame_valid_length = 7;
  f.data = {0x01, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "grasp_half_j1_6");

  f.frame_valid_length = 5;
  f.data = {0x04, 0xA0, 0xA0, 0xA0, 0xA0};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "grasp_half_j7_10");

  f.frame_valid_length = 7;
  f.data = {0x01, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "grasp_full_j1_6");

  f.frame_valid_length = 5;
  f.data = {0x04, 0x40, 0x40, 0x40, 0x40};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "grasp_full_j7_10");

  // 0x20：读五指法向压力（仅发指令字节）
  f.frame_valid_length = 1;
  f.data = {0x20};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "read_normal_pressure");

  // 抓握完成后回到完全张开（与初始 hand_open 相同：越大越远离掌心）
  f.frame_valid_length = 7;
  f.data = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "release_open_j1_6");

  f.frame_valid_length = 5;
  f.data = {0x04, 0xFF, 0xFF, 0xFF, 0xFF};
  SendSleepRecvOne(robot, f, ec, kRecvTimeoutMs, kSleepAfterSendMs, "release_open_j7_10");
}

int main(int argc, char *argv[]) {
  SetupConsoleUtf8();

  try {
    std::string ip = "192.168.71.161";
    std::string local = "192.168.71.161";
    if (argc >= 2) {
      ip = argv[1];
    }
    if (argc >= 3) {
      local = argv[2];
    }

    std::error_code ec;
    xMateErProRobot robot(ip, local);

    std::cout << "Connect Robot! ip=" << ip << " local=" << local << std::endl;
    // 默认 0x28（左手）；右手可传参：0x27
    int can_id = 0x28;
    if (argc >= 4) {
      can_id = static_cast<int>(std::strtol(argv[3], nullptr, 0));
    }
    std::cout << "CAN frame_id=0x" << std::hex << can_id << std::dec << std::endl;

    RunGraspDemo(robot, can_id, ec);
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
