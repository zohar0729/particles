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
        DummySensor() : std_err(0.1)
        {
            sensor_pose.x = 0.0;
            sensor_pose.y = 0.0;
            sensor_pose.theta = 0.0;
        }
        DummySensor(float x, float y, float theta) : std_err(0.1)
        {
            sensor_pose.x = x;
            sensor_pose.y = y;
            sensor_pose.theta = theta;
        }
        DummySensor(pose2d_t p) : sensor_pose(p){}
        float calcLikelihood(pose2d_t state)
        {
            return gaussian_function(PoseUtil::length(state, sensor_pose), std_err);
        }
        void setSensorParams(float x, float y, float theta, float std_err)
        {
            this->sensor_pose.x = x;
            this->sensor_pose.y = y;
            this->sensor_pose.theta = theta;
            this->std_err = std_err;
        }
    private:
        float gaussian_function(float x, float std_err)
        {
            return 1.0 / (sqrt(2 * M_PI) * std_err)*exp(-pow(x, 2.0) / (2 * pow(std_err, 2.0)));
        }
        pose2d_t sensor_pose;
        float std_err;
};

class Lidar : public Sensor
{
    public:
        Lidar()
            : z_hit(0.01), z_short(0.01), z_err(0.01), z_max(10) {}
        Lidar(float z_hit, float z_short, float z_err, float z_max)
            : z_hit(z_hit), z_short(z_short), z_err(z_err), z_max(z_max) {}
        ~Lidar()
        {
            delete[] this->ranges;
        }
        float calcLikelihood(pose2d_t state)
        {
            float z, phi = angle_min;
            float act_z, est_z, err;
            float result = 0.0;
            int c, r;
            for(int i = 0; i < nBeams; i++)
            {
                // 見えるであろうスキャン結果を得る
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

                // ビームごとに尤度計算
                act_z = ranges[i];
                est_z = z;
                err = act_z - est_z;
                result += 1/sqrt(2*M_PI * z_hit * z_hit)*exp(-(err * err)/(2 * z_hit * z_hit));
                if(act_z < est_z)
                {
                    result += z_short / (1.0f - exp(-z_short * est_z)) * exp(-z_short * est_z);
                }
                if(act_z > z_max){
                    result += z_err;
                }
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
        }
        void updateMap(OccupancyGrid *grid)
        {
            this->map = grid;
        }
        void setSensorParams(float z_hit, float z_short, float z_max, float z_err)
        {
            this->z_hit = z_hit;
            this->z_short = z_short;
            this->z_max = z_max;
            this->z_err = z_err;
        }
    private:
        float gaussian_function(float x, float std_err)
        {
            return 1.0 / (sqrt(2 * M_PI) * std_err)*exp(-pow(x, 2.0) / (2 * pow(std_err, 2.0)));
        }

        float angle_min, angle_max, angle_increment;
        float lidar_x, lidar_y, theta;
        int nBeams;
        float *ranges;

        float z_hit, z_short, z_max, z_err;

        OccupancyGrid *map;
};

#endif 