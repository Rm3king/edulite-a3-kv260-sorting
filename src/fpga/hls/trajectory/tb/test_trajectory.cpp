#include "../source/trajectory.hpp"
#include <cmath>
#include <cstdio>
#include <random>
#include "arm_vectors.hpp"
int main() {
 joint_t c[7][4],lo[7],hi[7],q[7];ap_uint<7> rejected;
 std::mt19937 rng(260);std::uniform_real_distribution<double>d(-3,3);
 double worst=0;int checks=0;
 for(int sample=0;sample<1000;++sample) {
  phase_t t=double(sample%101)/100;
  for(int a=0;a<7;++a){lo[a]=-4;hi[a]=4;for(int j=0;j<4;++j)c[a][j]=d(rng);}
  trajectory_eval(c,t,lo,hi,q,rejected);
  for(int a=0;a<7;++a){
   double x=t.to_double();double y=((c[a][3].to_double()*x+c[a][2].to_double())*x+c[a][1].to_double())*x+c[a][0].to_double();
   bool bad=y<lo[a].to_double()||y>hi[a].to_double();
   if(bool(rejected[a])!=bad){std::printf("FAIL classification\n");return 1;}
   if(bad){if(q[a]!=0)return 2;}
   else {double error=std::abs(q[a].to_double()-y);if(error>worst)worst=error;if(error>1e-6)return 3;}
   ++checks;
  }
 }
 for(int a=0;a<7;++a){lo[a]=-4;hi[a]=4;for(int j=0;j<4;++j)c[a][j]=0;}
 c[0][0]=4;c[1][0]=-4;trajectory_eval(c,phase_t(1),lo,hi,q,rejected);
 if(rejected!=0||q[0]!=4||q[1]!=-4)return 4;
 trajectory_eval(c,phase_t(1.5),lo,hi,q,rejected);if(rejected!=127)return 5;
 lo[3]=5;trajectory_eval(c,phase_t(0),lo,hi,q,rejected);if(rejected!=8)return 6;
 // Actual ARM planner coefficients, including captured joint coordinates,
 // synchronized MoveJ and braking candidates. Compare to double reference.
 double arm_worst=0;int arm_checks=0,arm_rejections=0;
 for(int sample=0;sample<arm_case_count;++sample) {
  for(int a=0;a<7;++a) {
   lo[a]=arm_lower[a];hi[a]=arm_upper[a];
   for(int j=0;j<4;++j)c[a][j]=arm_coefficients[sample][a][j];
  }
  for(int step=0;step<=100;++step) {
   phase_t phase=double(step)/100.;trajectory_eval(c,phase,lo,hi,q,rejected);
   for(int a=0;a<7;++a) {
    double u=phase.to_double();const double* cc=arm_coefficients[sample][a];
    double expected=((cc[3]*u+cc[2])*u+cc[1])*u+cc[0];
    double quantized=((c[a][3].to_double()*u+c[a][2].to_double())*u+c[a][1].to_double())*u+c[a][0].to_double();
    bool outside=quantized<lo[a].to_double()-1e-10||quantized>hi[a].to_double()+1e-10;
    if(bool(rejected[a])!=outside && std::abs(quantized-lo[a].to_double())>1e-10 && std::abs(quantized-hi[a].to_double())>1e-10)return 7;
    if(rejected[a]) {++arm_rejections;continue;}
    double error=std::abs(q[a].to_double()-expected);
    if(error>arm_worst)arm_worst=error;
    if(error>1e-6){std::printf("FAIL ARM/HLS case=%d axis=%d phase=%d error=%.12g\n",sample,a,step,error);return 8;}
    ++arm_checks;
   }
  }
 }
 std::printf("ARM_HLS_PASS cases=%d accepted_checks=%d rejected_samples=%d max_abs_error_rad=%.12g; no motor execution\n",arm_case_count,arm_checks,arm_rejections,arm_worst);
 std::printf("TEST_PASS trajectory checks=%d max_abs_error_rad=%.12g\n",checks+9,worst);return 0;
}
