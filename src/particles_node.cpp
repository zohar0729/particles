#include "particles_node.h"

#ifndef SINGLE_TEST
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map.h>
#include <sensor.h>
#include <motion.h>

class MCLProcessor
{
    public:
        MCLProcesser() :
            n_samples(1000),
            n_sensors(1)
        {
            prior = new particle_t[n_samples];
            posterior = new particle_t[n_samples];   
            map = new OccupancyGrid;
            sensors = new Lidar;
            model = new Omni;
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
            // TODO: オドメトリ受信時の処理
        }
    private:
        // ROS関係
        ros::Nodehandle private_nh;
        // 初期分布、提案分布、事後分布
        particle_t *prior, *posterior;
        int n_samples;

        // 地図、動作・計測モデル
        Map *map;
        Sensor *sensors;
        Motion *model;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particles");
    ros::NodeHandle nh;
    MCLProcessor p;

    // トピックの購読
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber odom_sub = nh.subscribe("/robot_encoder", 10, odomCallback);

    // 配信トピックの宣伝
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

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
    RandomGenerator g;
    pf p(&g);
    pose2d_t initial, std_err, odom;
    Omni omni(0.1, 0.1, 0.06, 0.06);
    DummySensor lrf(2.0, 1.0, 0.0);

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
#ifdef DEBUG
    printf("初期分布の生成に成功しました\n");
#endif
    omni.setOdometry(odom);
    p.createProposalDistribution(&omni);
#ifdef DEBUG
    printf("事前分布の生成に成功しました\n");
#endif
    p.createPosteriorDistribution(&lrf);
#ifdef DEBUG
    printf("事後分布の生成に成功しました\n");
#endif
    p.displaySamples(POSTERIOR_DISTRIBUTION);
}

#endif