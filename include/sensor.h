#ifndef DEF_SENSOR_H
#define DEF_SENSOR_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <map.h>
#include <util/pose.h>

class Sensor 
{
    public:
        virtual float calcLikelihood(pose2d_t) = 0;
};

class DummySensor : public Sensor
{
    public:
        DummySensor()
        {
            sensor_pose.x = 0.0;
            sensor_pose.y = 0.0;
            sensor_pose.theta = 0.0;
        }
        DummySensor(float x, float y, float theta)
        {
            sensor_pose.x = x;
            sensor_pose.y = y;
            sensor_pose.theta = theta;
        }
        DummySensor(pose2d_t p) : sensor_pose(p){}
        float calcLikelihood(pose2d_t state)
        {
            return gaussian_function(PoseUtil::length(state, sensor_pose), 0.1);
        }
    private:
        float gaussian_function(float x, float std_err)
        {
            return 1.0 / (sqrt(2 * M_PI) * std_err)*exp(-pow(x, 2.0) / (2 * pow(std_err, 2.0)));
        }
        pose2d_t sensor_pose;
};

class Lidar : public Sensor
{
    public:
        Lidar(){}
        ~Lidar()
        {
#ifdef DEBUG
            printf("LIDARのデストラクタ呼び出し\n");
#endif
            delete[] this->ranges;
        }
        float calcLikelihood(pose2d_t state)
        {
            float z, phi = angle_min;
            int c, r;
            for(int i = 0; i < nBeams; i++)
            {
                z = map->getResolution();
                c = ( lidar_x + z * cos(theta + phi) - map->getOriginX()) / map->getResolution();
                r = (-lidar_y - z * sin(theta + phi) + map->getOriginY()) / map->getResolution();
                while(!map->isOccupied(r, c))
                {
                    z += map->getResolution();
                    c = ( lidar_x + z * cos(theta + phi) - map->getOriginX()) / map->getResolution();
                    r = (-lidar_y - z * sin(theta + phi) + map->getOriginY()) / map->getResolution();
                }
                printf("%f %f\n", (theta + phi) , z);
                phi += (angle_max - angle_min) / nBeams;
            }
        }
        void updateStatus(const sensor_msgs::LaserScan& msg)
        {
            delete[] this->ranges;
            this->angle_min = msg.angle_min;
            this->angle_max = msg.angle_max;
            this->angle_increment = msg.angle_increment;
            this->nBeams = msg.ranges.size();
            this->ranges = new float[nBeams];
            for(int i = 0; i < nBeams; i++)
            {
                this->ranges[i] = msg.ranges[i];
            }
#ifdef DEBUG
            ROS_INFO("Received %d beams.", this->nBeams);
#endif
        }
        void updateMap(OccupancyGrid *grid)
        {
            this->map = grid;
        }
    private:
        float angle_min, angle_max, angle_increment;
        float lidar_x, lidar_y, theta;
        int nBeams;
        float *ranges;

        OccupancyGrid *map;
};

#endif 