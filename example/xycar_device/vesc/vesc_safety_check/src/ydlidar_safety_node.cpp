#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

struct RECT
{ 
    float xmin;
    float ymin;
    float xmax;
    float ymax;
};

const float PI = 3.1415926;

ros::Subscriber lidar_subscribe;
ros::Subscriber unsafe_ackermann;
ros::Publisher safety_ackermann;

int data_cnt;
float angle_increment = 0.0;

std::vector<float> lidar_ranges;
std::vector<float> xs(505, 0.0);
std::vector<float> ys(505, 0.0);

float steering = 0.0;
float steering_angle_velocity = 0.0;
float speed = 0.0;
float acceleration = 0.0;
float jerk = 0.0;

ackermann_msgs::AckermannDriveStamped safety_msg;

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    lidar_ranges = scan->ranges;
    data_cnt = lidar_ranges.size();
    angle_increment = scan->angle_increment;
}

void UnsafeAckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& unsafe)
{
    steering = unsafe->drive.steering_angle;
    steering_angle_velocity = unsafe->drive.steering_angle_velocity;
    speed = unsafe->drive.speed;
    acceleration = unsafe->drive.acceleration;
    jerk = unsafe->drive.jerk;
}

bool ObstacleCheck(RECT chkRange, int chkLimits)
{
    float angle;
    int dot = 0;

    for (int i=0; i < data_cnt; i++)
    {
        angle = angle_increment * i;
        xs[i] = lidar_ranges[i] * sin(angle);
        ys[i] = lidar_ranges[i] * cos(angle);

        if (((xs[i] >= chkRange.xmin) && (xs[i] <= chkRange.xmax)) && ((ys[i] >= chkRange.ymin) && (ys[i] <= chkRange.ymax)))
            dot++;
    }

    return dot > chkLimits;
}

void MsgCommon()
{
    safety_msg.header.frame_id = "odom";
    safety_msg.header.stamp = ros::Time::now();
    safety_msg.drive.steering_angle = steering;
}

void ForcedStop()
{
    MsgCommon();
    safety_msg.drive.speed = 0.0;
}

void MsgChk()
{
    MsgCommon();
    safety_msg.drive.speed = speed;
}

int main(int argc, char **argv)
{
    float basex = 0.22;
    float basey = 0.31;

    int hz;

    ros::init(argc, argv, "ydlidar_safety_node");
    ros::NodeHandle nh;

    nh.param<int>("hz", hz, 25);
    ros::Rate loop_rate(hz);

    lidar_subscribe = nh.subscribe("scan", 1, LidarCallback);
    unsafe_ackermann = nh.subscribe("ackermann_cmd", 1, UnsafeAckermannCallback);
    safety_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>("safety_ackermann_cmd", 1);

    RECT chkRange;

    bool obstacle = false; 
    bool last_obstacle = false;
    bool wait = false;
    double begin = 0.0;

    float max_interval = 0.0;

    int max_cnt = 114;
    int min_cnt = 0;
    float stop_width = 0.02;
    int Limits = 2;

    sleep(2);

    while (ros::ok())
    {
        MsgChk();
        obstacle = false;

        if (speed > 0) chkRange = { -basex, 0.1, basex, basey + speed };
        else if (speed < 0) chkRange = { -basex, -basey - speed, basex, -0.1 };
        else chkRange = { -basex, -basey, basex, basey };

        if (angle_increment != 0)
        {
            max_interval = (2.0 * ((float)std::abs(chkRange.ymax - chkRange.ymin) + 0.1) * PI) / 505.0;
            min_cnt = (int)(stop_width / max_interval);
            if (min_cnt < Limits) min_cnt = Limits;
        }

        obstacle = ObstacleCheck(chkRange, min_cnt);

        if ((!wait) && obstacle) 
        { 
            ForcedStop();
        }
        else if ((!wait) && ((last_obstacle) && (obstacle != last_obstacle))) wait = true;
        else if (wait && (obstacle && (obstacle == last_obstacle))) begin = ros::Time::now().toSec();

        if (wait && !begin) 
        {
            begin = ros::Time::now().toSec();
            ForcedStop();
        }
        else if (wait && begin && ((ros::Time::now().toSec() - begin) < 1.0)) ForcedStop();
        else if ((wait && obstacle) || (wait && begin && ((ros::Time::now().toSec() - begin) >= 1.0)))
        {
            wait = false;
            begin = 0.0;
        }

        safety_ackermann.publish(safety_msg);
        last_obstacle = obstacle;
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
