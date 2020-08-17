
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//需要实现“odom”参考系到“base_link”参考系的变换，以及nav_msgs/Odometry消息的发布

int main(int argc, char** argv)
{
    //定义一个消息发布者来发布“odom”消息，在定义一个tf广播，来发布tf变换信息
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    // 默认机器人的起始位置是odom参考系下的0点    
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    // 让机器人的base_link参考系在odom参考系下以x轴方向0.1m/s，Y轴速度-0.1m/s，角速度0.1rad/s的状态移动
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    //使用1Hz的频率发布odom消息，在实际系统中，往往需要更快的速度进行发布
    ros::Rate r(1.0);
    while(n.ok())
    {
    ros::spinOnce();              
    current_time = ros::Time::now();

    //积分计算里程计信息
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //为了兼容二维和三维的功能包，让消息结构更加通用，里程计的偏航角需要转换成四元数才能发布，辛运的是，ROS为我们提供了偏航角与四元数相互转换的功能
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    r.sleep();

    }
}