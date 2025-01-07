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

        pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // 创建定时器，指定频率50Hz
        timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &TrajectoryPublisher::publishTrajectory, this);
    }

    // 发布轨迹点
    void publishTrajectory(const ros::TimerEvent&)
    {
        static double               t = 0.0;
        mavros_msgs::PositionTarget target;

        // 根据轨迹类型生成目标点
        if (traj_type_ == S_SHAPE)
        {
            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target.type_mask        = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            // 生成S形轨迹：简单的S形曲线
            target.position.x = 5.0 * sin(t);  // S形曲线X坐标
            target.position.y = t;             // Y坐标随时间变化
            target.position.z = 10.0;          // 固定高度
        }
        else if (traj_type_ == O_SHAPE)
        {
            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target.type_mask        = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            // 生成O形轨迹：圆形轨迹
            target.position.x = 5.0 * cos(t);  // 圆形曲线X坐标
            target.position.y = 5.0 * sin(t);  // 圆形曲线Y坐标
            target.position.z = 10.0;          // 固定高度
        }

        pub_.publish(target);

        t += 0.02;  // 增加时间，模拟轨迹上的点

        if (t > 2 * M_PI)  // 限制t值，避免无限增大
        {
            t = 0.0;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Timer      timer_;  // 定时器对象

    TrajectoryType traj_type_ = S_SHAPE;  // 默认轨迹类型
    double         frequency_ = 50.0;     // 默认频率50Hz
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_test_node");

    TrajectoryPublisher trajectory_publisher;

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
