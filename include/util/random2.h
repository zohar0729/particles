#ifndef DEF_RANDOM2_H
#define DEF_RANDOM2_H

#include <math.h>
#include <time.h>

/* Period parameters */  
#define MT_N 624
#define MT_M 397
#define MATRIX_A 0x9908b0dfUL   /* constant vector a */
#define UPPER_MASK 0x80000000UL /* most significant w-r bits */
#define LOWER_MASK 0x7fffffffUL /* least significant r bits */

class MT 
{
    private:
        unsigned long mt[MT_N]; /* the array for the state vector  */
        int mti; /* mti==MT_N+1 means mt[MT_N] is not initialized */
    public:
    MT()
    {
        mti = MT_N + 1;
    }
    /* initializes mt[MT_N] with a seed */
    void init_genrand(unsigned long s)
    {
        mt[0]= s & 0xffffffffUL;
        for (mti=1; mti<MT_N; mti++) {
            mt[mti] = 
            (1812433253UL * (mt[mti-1] ^ (mt[mti-1] >> 30)) + mti); 
            /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
            /* In the previous versions, MSBs of the seed affect   */
            /* only MSBs of the array mt[].                        */
            /* 2002/01/09 modified by Makoto Matsumoto             */
            mt[mti] &= 0xffffffffUL;
            /* for >32 bit machines */
        }
    }
    /* initialize by an array with array-length */
    /* init_key is the array for initializing keys */
    /* key_length is its length */
    /* slight change for C++, 2004/2/26 */
    void init_by_array(unsigned long init_key[], int key_length)
    {
        int i, j, k;
        init_genrand(19650218UL);
        i=1; j=0;
        k = (MT_N>key_length ? MT_N : key_length);
        for (; k; k--) {
            mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1664525UL))
            + init_key[j] + j; /* non linear */
            mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
            i++; j++;
            if (i>=MT_N) { mt[0] = mt[MT_N-1]; i=1; }
            if (j>=key_length) j=0;
        }
        for (k=MT_N-1; k; k--) {
            mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1566083941UL))
            - i; /* non linear */
            mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
            i++;
            if (i>=MT_N) { mt[0] = mt[MT_N-1]; i=1; }
        }

        mt[0] = 0x80000000UL; /* MSB is 1; assuring non-zero initial array */ 
    }
    /* generates a random number on [0,0xffffffff]-interval */
    unsigned long genrand_int32(void)
    {
        unsigned long y;
        static unsigned long mag01[2]={0x0UL, MATRIX_A};
        /* mag01[x] = x * MATRIX_A  for x=0,1 */

        if (mti >= MT_N) { /* generate N words at one time */
            int kk;

            if (mti == MT_N+1)   /* if init_genrand() has not been called, */
                init_genrand(5489UL); /* a default initial seed is used */

            for (kk=0;kk<MT_N-MT_M;kk++) {
                y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
                mt[kk] = mt[kk+MT_M] ^ (y >> 1) ^ mag01[y & 0x1UL];
            }
            for (;kk<MT_N-1;kk++) {
                y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
                mt[kk] = mt[kk+(MT_M-MT_N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
            }
            y = (mt[MT_N-1]&UPPER_MASK)|(mt[0]&LOWER_MASK);
            mt[MT_N-1] = mt[MT_M-1] ^ (y >> 1) ^ mag01[y & 0x1UL];

            mti = 0;
        }
    
        y = mt[mti++];

        /* Tempering */
        y ^= (y >> 11);
        y ^= (y << 7) & 0x9d2c5680UL;
        y ^= (y << 15) & 0xefc60000UL;
        y ^= (y >> 18);

        return y;
    }
    /* generates a random number on [0,0x7fffffff]-interval */
    long genrand_int31(void)
    {
        return (long)(genrand_int32()>>1);
    }
    /* generates a random number on [0,1]-real-interval */
    double genrand_real1(void)
    {
        return genrand_int32()*(1.0/4294967295.0); 
        /* divided by 2^32-1 */ 
    }
    /* generates a random number on [0,1)-real-interval */
    double genrand_real2(void)
    {
        return genrand_int32()*(1.0/4294967296.0); 
        /* divided by 2^32 */
    }
    /* generates a random number on (0,1)-real-interval */
    double genrand_real3(void)
    {
        return (((double)genrand_int32()) + 0.5)*(1.0/4294967296.0); 
        /* divided by 2^32 */
    }
    /* generates a random number on [0,1) with 53-bit resolution*/
    double genrand_res53(void) 
    { 
        unsigned long a=genrand_int32()>>5, b=genrand_int32()>>6; 
        return(a*67108864.0+b)*(1.0/9007199254740992.0); 
    } 
};



class RandomGenerator
{
    public:
        RandomGenerator(){}
        double Uniform( void ){
            static int check=0;
            if( check==0 ){ mt.init_genrand((unsigned long)time(NULL)); check=1; }
            
            return mt.genrand_real3();
        }
        /*Exponential distribution*/
        double rand_exp( double lambda ){
            return -log(Uniform())/lambda;
        }
        /*Normal distribution*/
        double rand_normal( double mu, double sigma ){
            double z=sqrt( -2.0*log(Uniform()) ) * sin( 2.0*M_PI*Uniform() );
            return mu + sigma*z;
        }
        /*
        float rand_normal(float mu, float sigma)
        {
            float z=sqrt(-2.0 * log(Uniform())) * sin(2.0 * M_PI * Uniform());
            return mu + sigma * z;
        }
        */
        /*Chi-squared distribution*/
        double rand_chi( int k ){
            int i;
            double z,w=0;
            
            for(i=0;i<k;i++){
                z=sqrt( -2.0*log(Uniform()) ) * sin( 2.0*M_PI*Uniform() );
                w+=z*z;
            }
            
            return w;
        }
        /*Cauchy distribution*/
        double rand_cauchy( double mu, double gamma){
            return mu + gamma*tan(M_PI*( Uniform()-0.5 ));
        }
        /*Gamma distribution*/
        double rand_gamma( double theta, double kappa ){
            
            int int_kappa;
            double frac_kappa;
                
            int_kappa  = (int)kappa;
            frac_kappa = kappa - (double)int_kappa;
                
            double u,uu;
            double b,p,x_frac,x_int;
            int i;
                
            /*integer part*/
            x_int=0;
            for(i=0;i<int_kappa;i++){
                x_int+=-log(Uniform()); // add expnential random number with mean 1
            }
                
            /*fractional part*/
            if( fabs(frac_kappa) < 0.01 ) x_frac=0;
            
            else{
                b=(exp(1.0)+frac_kappa)/exp(1.0);
                while(1){
                    
                    u=Uniform();
                    p=b*u;
                        
                    uu=Uniform();
                        
                    if(p<=1.0){
                        x_frac=pow(p,1.0/frac_kappa);
                        if(uu<=exp(-x_frac)) break;
                    }
                        
                    else{
                        x_frac=-log((b-p)/frac_kappa);
                        if(uu<=pow(x_frac,frac_kappa-1.0)) break;
                    }
                    
                }
            }
                
            return (x_int+x_frac)*theta;
        }
        /*Log-normal distribution*/
        double rand_Lnormal( double mu, double sigma ){
            double z= mu + sigma*sqrt(-2.0*log(Uniform()))*sin(2.0*M_PI*Uniform());//gauss random number
            return exp(z);
        }
        /*Inverse Gauss distribution*/
        double rand_Igauss( double mu, double lambda ){
            double x,y,w,z;
            x=sqrt(-2.0*log(Uniform()))*sin(2.0*M_PI*Uniform());//normal random number
            y=x*x; //chi-squared
            w= mu + 0.5*y*mu*mu/lambda -(0.5*mu/lambda)*sqrt(4.0*mu*lambda*y+mu*mu*y*y); 
            z=Uniform();
                
            if( z< mu/(mu+w) )   return w;
            else                return mu*mu/w;
        }
    private:
        MT mt;
};

#endif