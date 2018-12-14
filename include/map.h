#ifndef DEF_MAP_H
#define DEF_MAP_H

#include <nav_msgs/OccupancyGrid.h>

class Map 
{
    public:
        virtual void load(const nav_msgs::OccupancyGrid&) = 0;
};

class OccupancyGrid : public Map
{
    public:
        OccupancyGrid(){}
        ~OccupancyGrid()
        {
#ifdef DEBUG
            printf("占有格子地図のデストラクタ呼び出し\n");
#endif 
            delete[] data;
        }
        void load(const nav_msgs::OccupancyGrid& msg)
        {
            width = msg.info.width;
            height = msg.info.height;
            resolution = msg.info.resolution;
            origin_x = msg.info.origin.position.x;
            origin_y = msg.info.origin.position.y;
            printf("width = %d, height = %d, resolution = %f, origin=(%0.2f, %0.2f)\\",
                width, height, resolution, origin_x, origin_y);
            data = new int[width * height];
            for(int i = 0; i < width * height; i++)
            {
                data[i] = msg.data[i];
            }
            
        }
        bool isOccupied(int r, int c)
        {
            if(r < 0 || c < 0 || r >= height || c >= width) return true;
            if(data[r * width + c] == 100) return true;
            else return false;
        }
        bool isOccupied(float x, float y)
        {
            int c,r;
            c = (int)( (x - origin_x) / resolution);
            r = (int)(-(y - origin_y) / resolution);
            return isOccupied(r, c);
        }
        bool isFree(int r, int c)
        {
            if(r < 0 || c < 0 || r >= height || c >= width) return false;
            if(data[r * width + c] == 0) return true;
            else return false;
        }
        bool isFree(float x, float y)
        {
            int c,r;
            c = (int)( (x - origin_x) / resolution);
            r = (int)(-(y - origin_y) / resolution);
            return isFree(r, c);
        }
        int getWidth(){ return width; }
        int getHeight(){ return height; }
        float getResolution(){ return resolution; }
        float getOriginX(){ return origin_x; }
        float getOriginY(){ return origin_y; }
    private:
        int width, height;
        float resolution, origin_x, origin_y;
        int *data;
};

#endif