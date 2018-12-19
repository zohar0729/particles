#include "particles_node.h"
#include <stdio.h>

// プロットする用のデータファイルを作成する
void createTemporaryDataFile(float* data, int dimention, int length)
{
    FILE* fp;

    fp = fopen("temp.dat", "w");
    if(fp == NULL){
        fprintf(stderr, "Cannot open temp.dat");
        return;
    }
    for(int i = 0; i < length; i++)
    {
        for(int j = 0; j < dimention; j++)
        {
            fprintf(fp, "%f ", data[dimention * i + j]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

#ifndef SINGLE_TEST
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particles");
    ParticlesNode p;

    ros::spin();
    return 0;
}

#else
#include <util/random2.h>
#include <util/pose.h>
#include <time.h>
#include <unistd.h>
#include <pf.h>
#include <sensor.h>
#include <motion.h>

float alpha[4];
float initial_position_err, initial_rotate_err;
float x_range_min, x_range_max, y_range_min, y_range_max;
int n_samples;

void readParametersFromFile()
{
    FILE *fp = fopen("config.txt", "r");
    fscanf(fp, "alpha_1 = %f\n", &alpha[0]);
    fscanf(fp, "alpha_2 = %f\n", &alpha[1]);
    fscanf(fp, "alpha_3 = %f\n", &alpha[2]);
    fscanf(fp, "alpha_4 = %f\n", &alpha[3]);

    fscanf(fp, "x_range_min = %f\n", &x_range_min);
    fscanf(fp, "x_range_max = %f\n", &x_range_max);
    fscanf(fp, "y_range_min = %f\n", &y_range_min);
    fscanf(fp, "y_range_max = %f\n", &y_range_max);

    fscanf(fp, "initial_position_error = %f\n", &initial_position_err);
    fscanf(fp, "initial_rotate_error = %f\n", &initial_rotate_err);

    fscanf(fp, "n_samples = %d", &n_samples);
    
    fclose(fp);
}

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        printf("Usage: %s [odom_x] [odom_y] [odom_theta]\n", argv[0]);
        return -1;
    }
    readParametersFromFile();

    pf p(n_samples);
    pose2d_t initial, std_err, odom;
    float plot[4*10000];
    int len;
    FILE* gp;   // gnuplotへのパイプ

    gp = popen("gnuplot -persist", "w");
    if(gp == NULL)
    {
        fprintf(stderr, "Cannot Open gnuplot.");
        return -1;
    }
    fprintf(gp, "set size square\n");
    fprintf(gp, "set xrange [%f:%f]\n", x_range_min, x_range_max);
    fprintf(gp, "set yrange [%f:%f]\n", y_range_min, y_range_max);

    Omni omni(alpha[0], alpha[1], alpha[2], alpha[3]);
    DummySensor lrf(0.14, 0.0, 0.0, 0.008);

    initial.x = 0.0;
    initial.y = 0.0;
    initial.theta = 0.0;
    std_err.x = initial_position_err;
    std_err.y = initial_position_err;
    std_err.theta = initial_rotate_err;
    odom.x = atof(argv[1]);
    odom.y = atof(argv[2]);
    odom.theta = atof(argv[3]);

    p.createInitialDistribution(initial, std_err);
    len = p.displaySamples(PRIOR_DISTRIBUTION, plot);
    createTemporaryDataFile(plot, 4, len);
    fprintf(gp, "plot \'temp.dat\' with vector\n");
    fflush(gp);
    sleep(1);
    omni.setOdometry(odom);
    p.createProposalDistribution(&omni);
    len = p.displaySamples(PRIOR_DISTRIBUTION, plot);
    createTemporaryDataFile(plot, 4, len);
    fprintf(gp, "plot \'temp.dat\' with vector\n");
    fflush(gp);
    sleep(1);
    p.createPosteriorDistribution(&lrf);
    len = p.displaySamples(PRIOR_DISTRIBUTION, plot);
    createTemporaryDataFile(plot, 4, len);
    fprintf(gp, "plot \'temp.dat\' with vector\n");
    fflush(gp);

    pclose(gp);
}

#endif