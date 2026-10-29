/**
 * @file model.h
 * @brief xMateModel模型库
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_MODEL_H
#define ROKAEAPI_MODEL_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "data_types.h"

namespace rokae {

 // forward declarations
 class BaseRobot;
 class XService;
 struct Info;

 /**
  * @class BaseModel
  * @brief 模型通用类
  */
 class XCORE_API BaseModel : public Base<BaseModel>{
  public:

   /// @cond DO_NOT_DOCUMENT
   explicit BaseModel(std::shared_ptr<XService> rpc);
   virtual ~BaseModel();
   /// @endcond

   /**
    * @brief 计算笛卡尔位姿所有逆解结果。支持除xMateSR(XMS)之外的所有机型
    * @param[in] posture 笛卡尔位姿，法兰相对与基座标系。其它坐标系需自行转换。
    * @param[out] confs 对应的confdata，错误码为0时有效
    * @param[out] ec 错误码，含逆解计算失败错误：-50102奇异点 | -50114 超限位 | -50519 超范围 | -50002 其它逆解错误
    * @return 逆解结果，单位弧度，错误码为0时有效
    */
   std::vector<std::vector<double>> calcAllIkSolutions(const CartesianPosition &posture, std::vector<std::vector<int>> &confs, error_code &ec) noexcept;

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class Model_T
 * @brief 模型模板类
 * @tparam DoF 轴数
 */
 template<unsigned short DoF>
 class XCORE_API Model_T : public BaseModel {

  public:
   using BaseModel::BaseModel;

   /**
    * @brief 根据位姿计算逆解。逆解选解策略:
    *   1) 默认Conf关闭时，选择离当前位置最近的解
    *   2) 默认Conf打开时，用confData计算
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系。参考的坐标系是通过setToolset()设置的
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double, DoF> calcIk(CartesianPosition posture, error_code &ec) noexcept;

   /**
    * @brief 根据位姿计算给定工具工件坐标系下逆解。逆解选解策略:
    *   1) 默认Conf关闭时，选择离当前位置最近的解
    *   2) 默认Conf打开时，用confData计算
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系
    * @param[in] tool_set 工具工件坐标系
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double,DoF> calcIk(CartesianPosition posture,const Toolset &tool_set, error_code& ec) noexcept;

   /**
    * @brief 根据轴角度计算正解。
    * @param[in] joints 轴角度, 单位: 弧度
    * @param[out] ec 错误码
    * @return 机器人末端位姿，相对于外部参考坐标系。参考的坐标系是通过setToolset()设置的
    */
   CartesianPosition calcFk(const std::array<double, DoF> &joints, error_code &ec) noexcept;

   /**
    * @brief 根据轴角度计算给定工具工件坐标系下正解
    * @param joints 轴角度, 单位: 弧度
    * @param tool_set 工具工件坐标系
    * @param ec 错误码
    * @return 机器人末端位姿，相对于外部参考坐标系
    */
   CartesianPosition calcFk(const std::array<double,DoF> &joints,const Toolset &tool_set, error_code& ec) noexcept;

 };

 /**
  * @enum SegmentFrame
  * @brief 连杆标号
  */
 enum class SegmentFrame : unsigned {
   joint1 = 1, joint2 = 2, joint3 = 3, joint4 = 4, joint5 = 5,
   joint6 = 6, joint7 = 7, flange = 8, endEffector = 9, stiffness = 10 };

 /**
  * @enum TorqueType
  * @brief 力矩类型
  */
 enum class TorqueType {
   full,     ///< 关节力矩，由动力学模型计算得到
   inertia,  ///< 惯性力
   coriolis, ///< 科氏力
   friction, ///< 摩擦力
   gravity   ///< 重力
 };

#ifdef XMATEMODEL_LIB_SUPPORTED

 /**
  * @class xMateModel
  * @brief xMate模型库。支持的机型: xMateER系列, XMC7/12, XMS3/4/5。
  * 注意：对于XMS5机型，需要升级特殊版本的机型文件后方可使用模型库，否则计算结果是错误的，请联系珞石技术支持人员进行机型文件升级。
  * @note 重力方向在模型构造时自动从控制器读取（ix.coord.base_in_world），无需手动设置。
  *       如需更新重力方向（如修改了基坐标系并重启控制器后），重连控制器并重新获取 model() 实例即可。
  * @tparam DoF 轴数
  */
 template <unsigned short DoF>
 class XCORE_API xMateModel : public Model_T<DoF> {

  public:
   /**
    * @brief Create an xMateModel instance
    * @throw ExecutionException 加载模型失败
    */
   explicit xMateModel(std::shared_ptr<XService> rpc, const Info& info);
   ~xMateModel();

   /**
    * @brief 设置负载参数，只在计算时使用，并不将参数传给机器人控制器，设置后动力学计算结果相应改变
    * @param[in] mass 质量
    * @param[in] cog 质心, 单位: m
    * @param[in] inertia 惯量
    * @see Load
    */
   void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   /**
    * @brief 设置TCP工具，只在计算时使用，并不将参数传给机器人控制器，设置TCP后，正逆解结果和输入参数相应改变
    * @param[in] f_t_ee 末端执行器相对于法兰的位姿
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿
    */
   void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k);

   /**
    * @brief 获取笛卡尔空间位置
    * @param[in] jntPos 需要计算笛卡尔位姿的关节角度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 向量化4x4位姿矩阵，行优先.
    */
   std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 计算结果
    */
   std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos, const std::array<double, DoF> &jntVel,
                                    SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间加速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] jntAcc 需要计算笛卡尔空间速度的关节角加速度
    * @param[in] nr 指定坐标系
    * @return 计算结果
    */
   std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    SegmentFrame nr = SegmentFrame::flange );

   /**
    * @brief 逆解获得关节空间位置。一个位姿可能对应多个关节角度，jntPos的选取原则是选取一个到该点关节角度变化最小的解。
    * @param[in] cartPos 法兰笛卡尔空间位姿
    * @param[in] elbow 臂角
    * @param[in] jntInit 初始关节角度
    * @param[out] jntPos 关节空间位置
    * @return 计算逆解结果 
    *    1) 0：逆解成功
    *    2) 1：逆解失败，奇异点
    *    3) 2：逆解失败，超关节软限位
    *    4) 3：逆解失败，目标点超运动范围
    *    5) 4-12：逆解失败，其他原因
    */
    int getJointPosNearst(const std::array<double, 16> &cartPos,
                    double elbow,
                    const std::array<double, DoF> &jntInit,
                    std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间位置。只求得一个与jntInit相邻的解。
    * @param[in] cartPos 法兰笛卡尔空间位姿
    * @param[in] elbow 臂角
    * @param[in] jntInit 初始关节角度
    * @param[out] jntPos 关节空间位置
    * @return 计算逆解结果 
    *    1) 0：逆解成功
    *    2) 1：逆解失败，奇异点
    *    3) 2：逆解失败，超关节软限位
    *    4) 3：逆解失败，目标点超运动范围
    *    5) 4-12：逆解失败，其他原因
    */
    int getJointPos(const std::array<double, 16> &cartPos,
                    double elbow,
                    const std::array<double, DoF> &jntInit,
                    std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间速度
    * @param[in] cartVel 法兰笛卡尔空间速度
    * @param[in] jntPos 此时关节角度
    * @return 计算结果
    */
   std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间加速度
    * @param[in] cartAcc 法兰笛卡尔空间加速度
    * @param[in] jntPos 此时关节角度
    * @param[in] jntVel 此时关节角速度
    * @return 计算结果
    */
   std::array<double, DoF> getJointAcc(const std::array<double, 6> &cartAcc,
                                       const std::array<double, DoF> &jntPos,
                                       const std::array<double, DoF> &jntVel);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] f_t_ee 末端执行器相对于法兰坐标系的位姿.
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos,
                                      const std::array<double, 16> &f_t_ee,
                                      const std::array<double, 16> &ee_t_k,
                                      SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵时间导数, 行优先
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] nr 指定坐标系
    * @return 计算结果 \f$ \dot{J} \f$, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    * @note 雅克比导数描述了雅克比矩阵随时间的变化率，常用于加速度控制和动力学计算
    */
   std::array<double, DoF*6> jacobianDot(const std::array<double, DoF> &jntPos,
                                         const std::array<double, DoF> &jntVel,
                                         SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵时间导数, 行优先
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] f_t_ee 末端执行器相对于法兰坐标系的位姿
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿
    * @param[in] nr 指定坐标系
    * @return 计算结果 \f$ \dot{J} \f$, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobianDot(const std::array<double, DoF> &jntPos,
                                         const std::array<double, DoF> &jntVel,
                                         const std::array<double, 16> &f_t_ee,
                                         const std::array<double, 16> &ee_t_k,
                                         SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 由模型计算无摩擦力的关节力矩, 计算结果单位: Nm。如有负载，先通过setLoad()设置负载参数。
    * @note 原getTorque() 和 getTorqueWithFriction不再支持，计算关节力矩都用此接口
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[out] trq_full 总关节力矩
    * @param[out] trq_inertia 惯性力
    * @param[out] trq_coriolis 科氏力，包括离心力
    * @param[out] trq_gravity 重力矩
    */
   void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                            const std::array<double, DoF> &jntVel,
                            const std::array<double, DoF> &jntAcc,
                            std::array<double, DoF> &trq_full,
                            std::array<double, DoF> &trq_inertia,
                            std::array<double, DoF> &trq_coriolis,
                            std::array<double, DoF> &trq_gravity);

   /**
    * @brief 获取关节空间质量矩阵(惯性矩阵)。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @return 质量矩阵，行优先存储，长度为 \f$ \mathbb{R}^{DoF \times DoF} \f$
    * @note 质量矩阵是对称正定矩阵，描述了关节空间的惯性特性。
    *       矩阵元素 M(i,j) 在数组中的位置为 i*DoF + j
    */
   std::array<double, DoF*DoF> getMassMatrix(const std::array<double, DoF> &jntPos);

   /**
    * @brief 获取离心力和科里奥利力项。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节速度
    * @return 离心力和科里奥利力向量，长度为 \f$ \mathbb{R}^{DoF} \f$
    * @note 这是动力学方程 τ = M(q)q̈ + C(q,q̇) + G(q) 中的 C(q,q̇) 项，
    *       包含了离心力和科里奥利力的影响。
    */
   std::array<double, DoF> getCoriolisVector(const std::array<double, DoF> &jntPos, const std::array<double, DoF> &jntVel);

  XCORESDK_DECLARE_IMPLD
 };

 template <unsigned short DoF>
 using XMateModel = xMateModel<DoF>;

#endif // #ifdef XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif // ROKAEAPI_MODEL_H
