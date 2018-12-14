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
        pf(RandomGenerator *gen) : n_samples(1000), g(gen)
        {
            prior = new particle_t[n_samples];
            posterior = new particle_t[n_samples];
        }
        pf(RandomGenerator *gen, int n) : n_samples(n), g(gen)
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
                prior[i].state.x = g->rand_normal(initial_pose.x, std_err.x);
                prior[i].state.y = g->rand_normal(initial_pose.y, std_err.y);
                prior[i].state.theta = g->rand_normal(initial_pose.theta, std_err.theta);
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

            for(i = 0; i < n_samples; i++)
            {
                prior[i].likelihood = model->calcLikelihood(prior[i].state);
                total_likelihood += prior[i].likelihood;
            }
#ifdef DEBUG
            printf("total_likelihood = %f\n", total_likelihood);
#endif
            sortSamplesByLikelihood(prior);
#ifdef DEBUG
            printf("パーティクルのソートに成功\n");
#endif
            for(i = 0; i < n_samples; i++)
            {
                cursol = g->Uniform() * total_likelihood;
                for(j = 0; (cursol > 0 && j < n_samples); j++)
                {
                    cursol -= prior[j].likelihood;
                }
                posterior[i] = prior[j];
            }
        }
        void displaySamples(int flag)
        {
            for(int i = 0; i < n_samples; i++)
            {
                if(flag == PRIOR_DISTRIBUTION)
                {
                    printf("%f %f %f %f\n", 
                        prior[i].state.x, prior[i].state.y, 
                        0.1 * cos(prior[i].state.theta), 0.1 * sin(prior[i].state.theta));
                }
                else
                {
                    printf("%f %f %f %f\n", 
                        posterior[i].state.x, posterior[i].state.y, 
                        0.1 * cos(posterior[i].state.theta), 0.1 * sin(posterior[i].state.theta));
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
        RandomGenerator *g;
        // 事前分布、事後分布
        particle_t *prior, *posterior;
        int n_samples;
};

#endif