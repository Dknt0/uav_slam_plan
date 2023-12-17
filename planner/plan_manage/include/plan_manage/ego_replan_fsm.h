/**
 * ego_planner 规划状态机头文件
 * 
*/

#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

// 贝塞尔样条线优化
#include <bspline_opt/bspline_optimizer.h>
// 栅格地图
#include <plan_env/grid_map.h>
// 轨迹工具
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/DataDisp.h>
#include <traj_utils/planning_visualization.h>
// 轨迹规划管理
#include <plan_manage/planner_manager.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    // 状态机状态
    enum FSM_EXEC_STATE
    {
      INIT, // 初始化
      WAIT_TARGET, // 等待目标
      GEN_NEW_TRAJ, // 生成新轨迹
      REPLAN_TRAJ, // 重规划
      EXEC_TRAJ, // 执行轨迹
      EMERGENCY_STOP, // 紧急停止
      SEQUENTIAL_START // 顺序启动
    };
    // 目标类型
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1, // 手动目标  可以来自 Rviz 中 2d navgoal
      PRESET_TARGET = 2, // 现有目标
      REFENCE_PATH = 3 // 参考轨迹  未用到
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_; // 规划管理器
    PlanningVisualization::Ptr visualization_; // 规划显示
    traj_utils::DataDisp data_disp_; // 显示数据
    traj_utils::MultiBsplines multi_bspline_msgs_buf_; // 多样条线

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_; // 有无重规划阈值标志  重规划阈值
    double waypoints_[50][3]; // 路标点，最多50个  初始化时从参数服务器中获取
    int waypoint_num_; // 路标点总数
    int  wp_id_; //  路标点id
    double planning_horizen_, planning_horizen_time_; //
    double emergency_time_; //
    bool flag_realworld_experiment_; // 实物实验标志
    bool enable_fail_safe_; // 失效检查标志

    /* planning data */
    // 标志位    
    bool have_trigger_; // 存在触发器  遥控上的触发器，会在实物实验中用到
    bool have_target_; // 存在目标
    bool have_odom_; // 存在里程计   会在第一次收到里程计消息时设置
    bool have_new_target_; // 存在新目标
    bool have_recv_pre_agent_; // 存在？
    FSM_EXEC_STATE exec_state_; // 执行状态
    int continously_called_times_{0}; // 连续调用次数
    // 里程计信息  加速度没有用到
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_; // 里程计姿态
    // 起点，终点，路径点
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_; // 路标点，从 waypoints_ 导入
    int current_wp_; // 当前路标点序号

    bool flag_escape_emergency_; // 躲避紧急情况标志位置

    /* ROS utils */
    ros::NodeHandle node_; // 节点句柄
    ros::Timer exec_timer_, safety_timer_; // 执行计时器  安全计时器
    // 路标点接收者     swarm轨迹接收者  样条线接收者   触发器接收者
    ros::Subscriber waypoint_sub_; // 2d Nav goal 话题接收  仅当手动模式下存在
    ros::Subscriber odom_sub_; // 里程计接收
    ros::Subscriber swarm_trajs_sub_;
    ros::Subscriber broadcast_bspline_sub_;
    ros::Subscriber trigger_sub_; // 触发器
    // 重规划发布者   新发布者？   样条线发布者   数据显示发布者？  swarm轨迹发布者   样条线发布者
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg);
    void BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif