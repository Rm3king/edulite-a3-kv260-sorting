#include "../source/fk_batch.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
extern "C" int native_fk_batch(const int32_t*,const int32_t*,unsigned,int32_t*);
static int seed=12345;
static double rnd(){seed=(int)((1103515245u*(unsigned)seed+12345u)&0x7fffffff);return double(seed)/2147483647.;}
int main(){
 int co[48]={},box[6]={-16777216,16777216,-16777216,16777216,-16777216,16777216},a[32],b[32];
 double maxerr=0;int checks=0;
 for(int trial=0;trial<114;++trial){
  for(int j=0;j<6;++j)for(int k=0;k<8;++k)co[j*8+k]=(int)std::lrint(((k==0?(rnd()-.5)*5.:trial>=100?(rnd()-.5)*.03:0))*16777216.);
  unsigned n=trial<100?1:trial==113?1025:33;
  if(trial%3==0){box[5]=0;}else box[5]=16777216;
  fk_batch(co,box,n,a);native_fk_batch(co,box,n,b);
  for(int i=0;i<6;++i){if(a[i]!=b[i]){printf("count/status mismatch trial%d field%d %d %d\n",trial,i,a[i],b[i]);return 1;}++checks;}
  for(int i=6;i<24;++i){double err=std::abs(double(a[i])-b[i])/16777216.;if(err>maxerr)maxerr=err;if(err>2e-6){printf("numeric mismatch %d %d %.9g\n",trial,i,err);return 2;}++checks;}
 }
 for(unsigned n: {0u,4098u}){fk_batch(co,box,n,a);if(a[1]!=1 || a[2])return 3;}
 box[0]=2;box[1]=1;fk_batch(co,box,1,a);if(a[1]!=1 || a[2])return 4;
 box[0]=-16777216;box[1]=16777216;for(int i=0;i<48;++i)co[i]=0;co[0]=7*16777216;fk_batch(co,box,1,a);if(a[1]!=2 || !(a[5]&64))return 5;
 printf("TEST_PASS fk_batch checks=%d max_component_error=%.12g (metres or rotation entries); sampled checks only\n",checks,maxerr);return 0;
}
