#ifndef DEF_MOTION_H
#define DEF_MOTION_H

#include <util/pose.h>
#include <util/random2.h>

class Motion
{
    public:
        virtual pose2d_t sampleByMotionModel(pose2d_t state) = 0;
};

class Omni : public Motion
{
    public:
        Omni()
        {
            alpha[0] = 1.0;
            alpha[1] = 1.0;
            alpha[2] = 1.0;
            alpha[3] = 1.0;
        }
        Omni(float a, float b, float c, float d)
        {
            alpha[0] = a;
            alpha[1] = b;
            alpha[2] = c;
            alpha[3] = d;
        }
        // 動作モデルに基づいてパーティクルの位置を変化させる
        pose2d_t sampleByMotionModel(pose2d_t state)
        {
            float delta_rot[2], delta_trans;
            float _delta_rot[2], _delta_trans;
            pose2d_t result;

            delta_rot[0] = atan2(diff.y, diff.x);
            delta_rot[1] = diff.theta - delta_rot[0];
            delta_trans = sqrt(diff.x * diff.x + diff.y * diff.y);
            
            _delta_rot[0] = delta_rot[0] - g.rand_normal(0.0, alpha[0] * pow(delta_rot[0], 2.0) + alpha[1] * pow(delta_trans, 2.0));
            _delta_rot[1] = delta_rot[1] - g.rand_normal(0.0, alpha[0] * pow(delta_rot[1], 2.0) + alpha[1] * pow(delta_trans, 2.0));
            _delta_trans = delta_trans - g.rand_normal(0.0, alpha[2] * pow(delta_trans, 2.0) + alpha[3] * pow(delta_rot[0], 2.0) + alpha[3] * pow(delta_rot[1], 2.0));

            result.x = state.x + _delta_trans * cos(state.theta + _delta_rot[0]);
            result.y = state.y + _delta_trans * sin(state.theta + _delta_rot[0]);
            result.theta = state.theta + _delta_rot[0] + _delta_rot[1];

            return result;
        }
        void setOdometry(pose2d_t param)
        {
            diff = param;
        }
        void setMotionParams(float alpha_1, float alpha_2, float alpha_3, float alpha_4)
        {
            alpha[0] = alpha_1;
            alpha[1] = alpha_2;
            alpha[2] = alpha_3;
            alpha[3] = alpha_4;
        }
    private:
        RandomGenerator g;
        float alpha[4];
        pose2d_t diff;
};

#endif