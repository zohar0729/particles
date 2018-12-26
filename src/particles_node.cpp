#include "particles_node.h"
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pf.h>
#include <map.h>
#include <sensor.h>
#include <motion.h>

class ParticlesNode
{
    public:
        // コンストラクタ
        ParticlesNode() : isFirstMapReceived(false)
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

            odom_sub = nh.subscribe("/robot_odom", 100, &ParticlesNode::odomCallback, this);
            scan_sub = nh.subscribe("/scan", 100, &ParticlesNode::scanCallback, this);
            map_sub  = nh.subscribe("/map", 1, &ParticlesNode::mapCallback, this);
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

            delta.x = 0;
            delta.y = 0;
            delta.theta = 0;
        }
        // 地図を受け取った際の処理
        void mapCallback(const nav_msgs::OccupancyGrid& msg)
        {
            ROS_INFO("地図情報を受信しました。自己位置推定を開始できます");
            map.load(msg);          // 地図の本体に読み込み
            lrf.updateMap(&map);    // 観測モデルに地図のポインタを渡す
        }
        // スキャンデータを受け取った際の処理
        void scanCallback(const sensor_msgs::LaserScan& msg)
        {
            lrf.updateStatus(msg);
        }
        // オドメトリ入力を受け取った際の処理
        void odomCallback(const geometry_msgs::TwistStamped& msg)
        {
            static tf::TransformBroadcaster broadcaster;
            static tf::TransformListener listener;
            tf::Vector3 linear;
            float angular;
            tf::Quaternion q;
            tf::StampedTransform transform;

            try{
                listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            linear = transform.getOrigin();
            q = transform.getRotation();
            angular = tf::getYaw(q);
        }
            
    private:
        // ROS関係
        ros::Subscriber odom_sub, scan_sub, map_sub;
        ros::Publisher pose_pub;
        
        //パーティクルフィルタ本体
        pf p;
        // 地図、動作・計測モデル
        OccupancyGrid map;
        Lidar lrf;
        Omni omni;
        // 蓄積された姿勢の変位
        pose2d_t delta;
        // 自己位置推定を開始できるかどうか
        bool isFirstMapReceived;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particles");
    ParticlesNode p;

    ros::spin();
    return 0;
}