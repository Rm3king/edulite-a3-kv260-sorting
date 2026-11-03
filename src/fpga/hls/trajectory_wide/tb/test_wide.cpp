#include "../source/trajectory_wide.hpp"
#include <random>
#include <cmath>
#include <cstdio>
#include "wide_vectors.hpp"
int main(){
 joint_t c[7][8],lo[7],hi[7],q[7];ap_uint<7> rejected;
 std::mt19937 rng(2603);std::uniform_real_distribution<double>d(-.8,.8);
 double worst=0;int checks=0;int samples=1000,case_index=0,case_tick=0;
 for(int i=0;i<wide_case_count;++i)samples+=wide_ticks[i]+1;
 for(int sample=0;sample<samples;++sample){
  phase_t t=double(sample%101)/100.;
  if(sample>=1000)t.range(24,0)=ap_uint<25>((static_cast<unsigned long long>(case_tick)*16777216)/wide_ticks[case_index]);
  for(int a=0;a<7;++a){lo[a]=-4;hi[a]=4;
   for(int j=0;j<8;++j)c[a][j]=sample<1000 ? d(rng) : wide_coefficients[case_index][a][j];
   if(sample>=1000){lo[a]=wide_lower[a];hi[a]=wide_upper[a];}
  }
  trajectory_eval_wide(c,t,lo,hi,q,rejected);
  for(int a=0;a<7;++a){double y=0;for(int j=7;j>=0;--j)y=y*t.to_double()+c[a][j].to_double();
   bool bad=y<lo[a].to_double() || y>hi[a].to_double();
   if(bool(rejected[a])!=bad && std::min(std::abs(y-lo[a].to_double()),std::abs(y-hi[a].to_double()))>1e-10)return 1;
   if(!bad){double e=std::abs(q[a].to_double()-y);worst=std::max(worst,e);if(e>1e-6)return 2;}
   else if(q[a]!=0)return 3;
   ++checks;
  }
  if(sample>=1000 && ++case_tick>wide_ticks[case_index]){case_tick=0;++case_index;}
 }
 for(int a=0;a<7;++a){lo[a]=-4;hi[a]=4;for(int j=0;j<8;++j)c[a][j]=0;}
 c[0][0]=4;c[1][0]=-4;trajectory_eval_wide(c,phase_t(1),lo,hi,q,rejected);if(rejected || q[0]!=4 || q[1]!=-4)return 4;
 trajectory_eval_wide(c,phase_t(1.5),lo,hi,q,rejected);if(rejected!=127)return 5;
 lo[3]=5;trajectory_eval_wide(c,phase_t(0),lo,hi,q,rejected);if(rejected!=8)return 6;
 std::printf("TEST_PASS trajectory_wide checks=%d cases=%d max_abs_error_rad=%.12g\n",checks+9,wide_case_count,worst);return 0;
}
