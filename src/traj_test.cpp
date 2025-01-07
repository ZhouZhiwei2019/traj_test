#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

enum TrajectoryType
{
    S_SHAPE,
    O_SHAPE
};

class TrajectoryPublisher
{
public:
    TrajectoryPublisher()
    {
        // 获取参数
        int traj_type_int;
        ros::param::get("~traj_type", traj_type_int);             // 获取 traj_type 参数，暂存为 int
        traj_type_ = static_cast<TrajectoryType>(traj_type_int);  // 转换为枚举类型

        ros::param::get("~sample_frequency", frequency_);
        ros::param::get("~height", height_);
        ros::param::get("~radius", radius_);
        ros::param::get("~axis_vmax", axis_vmax_);          //单轴最大速度
        ros::param::get("~yaw_ctl", yaw_ctl_);              // 获取是否启用yaw控制的参数
        ros::param::get("~period_factor", period_factor_);  // 获取周期控制因子

        pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // 创建定时器，指定频率50Hz
        timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &TrajectoryPublisher::publishTrajectory, this);
    }

    // 发布轨迹点
    void publishTrajectory(const ros::TimerEvent&)
    {
        static double               t = 0.0;
        mavros_msgs::PositionTarget target;

        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask        = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
                           | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
                           | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // 根据轨迹类型生成目标点
        if (traj_type_ == S_SHAPE)
        {
            // 生成S形轨迹：简单的S形曲线
            target.position.x = axis_vmax_ * period_factor_ * radius_ * t;  // S形曲线X坐标，一直往前走 / (2 * M_PI)
            target.position.y = axis_vmax_ * radius_ * sin(t * 2 * M_PI);   // Y坐标随时间正弦变化
            target.position.z = height_;                                    // 固定高度
            target.yaw        = 0.0;

            // 计算S形轨迹的速度
            double v_x     = axis_vmax_ * period_factor_ * radius_;  // x轴速度是固定的
            double v_y     = axis_vmax_ * radius_ * 2 * M_PI * cos(t * 2 * M_PI);
            double v_z     = 0.0;
            double v_total = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);

            // 如果启用yaw控制，计算当前点的yaw
            if (yaw_ctl_)
            {
                // 计算S形轨迹的yaw角度，采用速度方向的角度（路径切线方向）
                double yaw = std::atan2(v_y, v_x);  // 计算当前点的yaw角
                target.yaw = yaw;
            }

            ROS_INFO("S-Shape Velocity: v_x = %f, v_y = %f, v_total = %f", v_x, v_y, v_total);
        }
        else if (traj_type_ == O_SHAPE)
        {
            // 生成O形轨迹：圆形轨迹
            target.position.x = axis_vmax_ * radius_ * cos(t * 2 * M_PI + 0.5 * M_PI);  // 圆形曲线X坐标，初始相位为0开始
            target.position.y = axis_vmax_ * radius_ * sin(t * 2 * M_PI);               // 圆形曲线Y坐标
            target.position.z = height_;                                                // 固定高度
            target.yaw        = 0.0;

            // 计算O形轨迹的速度
            double v_x     = axis_vmax_ * -radius_ * 2 * M_PI * sin(t * 2 * M_PI + 0.5 * M_PI);
            double v_y     = axis_vmax_ * radius_ * 2 * M_PI * cos(t * 2 * M_PI);
            double v_z     = 0.0;
            double v_total = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);

            // 如果启用yaw控制，计算指向圆心的yaw角
            if (yaw_ctl_)
            {
                // 计算圆心指向当前位置的yaw角度，指向原点(0, 0)
                double yaw = std::atan2(target.position.y, target.position.x);  // 计算圆心方向的yaw角
                target.yaw = yaw;
            }

            ROS_INFO("O-Shape Velocity: v_x = %f, v_y = %f, v_total = %f", v_x, v_y, v_total);
        }

        pub_.publish(target);

        // 更新时间步长，除以周期因子来调整周期
        t += 1 / (frequency_ * period_factor_);  // 增加时间，模拟轨迹上的点

        if (t > 2 * M_PI)  // 限制t值，避免无限增大
        {
            t = 0.0;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Timer      timer_;  // 定时器对象

    TrajectoryType traj_type_     = S_SHAPE;  // 默认轨迹类型
    double         frequency_     = 50.0;     // 默认频率50Hz
    double         height_        = 1.0;      // 默认高度1m
    double         radius_        = 1.0;      // 默认半径1m
    double         axis_vmax_     = 1.0;      // 默认单轴最大速度1m/s
    bool           yaw_ctl_       = false;    // 默认不启用yaw控制
    double         period_factor_ = 4.0;      // 默认周期因子为1，1s转2 * M_PI角度
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_test_node");

    TrajectoryPublisher trajectory_publisher;

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
