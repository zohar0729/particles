#include <stdio.h>
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
float lrf_x, lrf_y, lrf_theta, lrf_position_err;
int n_samples;

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

// 実験用パラメータをファイルから読み込む
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

    fscanf(fp, "lrf_x = %f\n", &lrf_x);
    fscanf(fp, "lrf_y = %f\n", &lrf_y);
    fscanf(fp, "lrf_theta = %f\n", &lrf_theta);
    fscanf(fp, "lrf_position_err = %f\n", &lrf_position_err);

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
    DummySensor lrf(lrf_x, lrf_y, lrf_theta, lrf_position_err);

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