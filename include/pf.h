#ifndef DEF_PF_H
#define DEF_PF_H

#include <util/pose.h>
#include <util/random2.h>
#include <map.h>
#include <motion.h>
#include <sensor.h>

typedef struct __particle
{
    pose2d_t state;
    float likelihood;
}particle_t;

enum target_distribution
{
    PRIOR_DISTRIBUTION = 0,
    POSTERIOR_DISTRIBUTION = 1
};

class pf
{
    public:
        pf() : n_samples(1000)
        {
            prior = new particle_t[n_samples];
            posterior = new particle_t[n_samples];
        }
        pf(int n) : n_samples(n)
        {
            prior = new particle_t[n_samples];
            posterior = new particle_t[n_samples];
        }
        ~pf()
        {
            delete[] prior;
            delete[] posterior;
        }
        // 初期分布を生成する
        void createInitialDistribution(pose2d_t initial_pose, pose2d_t std_err)
        {
            for(int i = 0; i < n_samples; i++)
            {
                prior[i].state.x = g.rand_normal(initial_pose.x, std_err.x);
                prior[i].state.y = g.rand_normal(initial_pose.y, std_err.y);
                prior[i].state.theta = g.rand_normal(initial_pose.theta, std_err.theta);
            }
        }
        // 動作モデルに従って事前分布を生成する
        void createProposalDistribution(Motion *model)
        {
            for(int i = 0; i < n_samples; i++)
            {
                prior[i].state = model->sampleByMotionModel(prior[i].state);
            }
        }
        // リサンプリングによって事後分布を生成する
        void createPosteriorDistribution(Sensor *model)
        {
            float total_likelihood = 0.0;
            float cursol;
            int i, j;

            // 尤度計算
            for(i = 0; i < n_samples; i++)
            {
                prior[i].likelihood = model->calcLikelihood(prior[i].state);
                total_likelihood += prior[i].likelihood;
            }
            sortSamplesByLikelihood(prior);
            // リサンプリング
            for(i = 0; i < n_samples; i++)
            {
                cursol = g.Uniform() * total_likelihood;
                for(j = 0; (cursol > 0 && j < n_samples); j++)
                {
                    cursol -= prior[j].likelihood;
                }
                posterior[i] = prior[j];
            }
            // 事前分布にコピー
            for(i = 0; i < n_samples; i++)
            {
                prior[i] = posterior[i];
            }
        }
        // パーティクルの状態を配列の形で返す
        int displaySamples(int flag, float* samples)
        {
            for(int i = 0; i < n_samples; i++)
            {
                if(flag == PRIOR_DISTRIBUTION)
                {
                    samples[4 * i + 0] = prior[i].state.x;
                    samples[4 * i + 1] = prior[i].state.y;
                    samples[4 * i + 2] = 0.02 * cos(prior[i].state.theta);
                    samples[4 * i + 3] = 0.02 * sin(prior[i].state.theta);
                }
                else
                {
                    samples[4 * i + 0] = posterior[i].state.x;
                    samples[4 * i + 1] = posterior[i].state.y;
                    samples[4 * i + 2] = 0.02 * cos(posterior[i].state.theta);
                    samples[4 * i + 3] = 0.02 * sin(posterior[i].state.theta);
                }
            }
        }
    private:
        // 尤度順(降順)にパーティクルを整列する
        void sortSamplesByLikelihood(particle_t *samples)
        {
            int i, j;
            for(i = 0; i < n_samples; i++)
            {
                j = i;
                while(j > 0 && samples[j-1].likelihood < samples[j].likelihood)
                {
                    swapSamples(&samples[j-1], &samples[j]);
                    j--;
                }
            }
        }
        // パーティクルを交換する
        void swapSamples(particle_t *to, particle_t *from)
        {
            particle_t tmp;
            tmp = *to;
            *to = *from;
            *from = tmp;
        }
        // 乱数生成器
        RandomGenerator g;
        // 事前分布、事後分布
        particle_t *prior, *posterior;
        int n_samples;
};

#endif