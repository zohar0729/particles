#include "particles_node.h"

#ifndef SINGLE_TEST
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pf.h>
#include <map.h>
#include <sensor.h>
#include <motion.h>

class ParticlesNode
{
    public:
        ParticlesNode()
        {
            ros::NodeHandle nh("~");
            pose2d_t initial_pose_, std_err_;
            float alpha_1_, alpha_2_, alpha_3_, alpha_4_;
            float z_hit_, z_short_, z_max_, z_err_;

            nh.param<float>("initial_pose_x", initial_pose_.x, 0.0);
            nh.param<float>("initial_pose_y", initial_pose_.y, 0.0);
            nh.param<float>("initial_pose_theta", initial_pose_.theta, 0.0);
            nh.param<float>("initial_stderr_x", std_err_.x, 0.1);
            nh.param<float>("initial_stderr_y", std_err_.y, 0.1);
            nh.param<float>("initial_stderr_theta", std_err_.theta, 0.1);

            nh.param<float>("alpha_1", alpha_1_, 0.1);
            nh.param<float>("alpha_2", alpha_2_, 0.1);
            nh.param<float>("alpha_3", alpha_3_, 0.1);
            nh.param<float>("alpha_4", alpha_4_, 0.1);

            nh.param<float>("z_hit", z_hit_, 0.03);
            nh.param<float>("z_short", z_short_, 0.2);
            nh.param<float>("z_max", z_max_, 30.0);
            nh.param<float>("z_err", z_err_, 0.01);

            omni.setMotionParams(alpha_1_, alpha_2_, alpha_3_, alpha_4_);
            lrf.setSensorParams(z_hit_, z_short_, z_max_, z_err_);
        }
        void mapCallback(const nav_msgs::OccupancyGrid& msg)
        {
            ROS_INFO("地図情報を受信しました。自己位置推定を開始できます");
            map.load(msg);          // 地図の本体に読み込み
            lrf.updateMap(&map);    // 観測モデルに地図のポインタを渡す
        }
        void scanCallback(const sensor_msgs::LaserScan& msg)
        {
            lrf.updateStatus(msg);
        }
        void odomCallback(const geometry_msgs::TwistStamped& msg)
        {
            pose2d_t odom;
            odom.x = msg.twist.linear.x;
            odom.y = msg.twist.linear.y;
            odom.theta = msg.twist.angular.z;
            omni.setOdometry(odom);
        }
    private:
        //パーティクルフィルタ本体
        pf p;
        // 地図、動作・計測モデル
        OccupancyGrid map;
        Lidar lrf;
        Omni omni;
        // 蓄積された姿勢の変位
        pose2d_t delta;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particles");
    ParticlesNode p;

    ros::spin();
    return 0;
}

#else
#include <stdio.h>
#include <util/random2.h>
#include <util/pose.h>
#include <time.h>
#include <pf.h>
#include <sensor.h>
#include <motion.h>

int main(int argc, char** argv)
{
    pf p;
    pose2d_t initial, std_err, odom;
    Omni omni(0.1, 0.1, 0.06, 0.06);
    DummySensor lrf(2.0, 0.0, 0.0);

    if(argc < 4)
    {
        printf("Usage: %s [odom_x] [odom_y] [odom_theta]\n", argv[0]);
        return -1;
    }
    initial.x = 0.0;
    initial.y = 0.0;
    initial.theta = 0.0;

    std_err.x = 0.0;
    std_err.y = 0.0;
    std_err.theta = 0.0;

    odom.x = atof(argv[1]);
    odom.y = atof(argv[2]);
    odom.theta = atof(argv[3]);

    p.createInitialDistribution(initial, std_err);
    omni.setOdometry(odom);
    p.createProposalDistribution(&omni);
    p.createPosteriorDistribution(&lrf);
    p.displaySamples(POSTERIOR_DISTRIBUTION);
}

#endif