#include <cstdint>
#include <cmath>
#include <algorithm>
#include "model_constants.hpp"
static void pose(const double q[6],double T[12]) {
 double R[3][3]={{1,0,0},{0,1,0},{0,0,1}},p[3]={0,0,0};
 for(int j=0;j<6;++j){
  double O[3][3],s=std::sin(q[j]),c=std::cos(q[j]);
  for(int a=0;a<3;++a){
   for(int b=0;b<3;++b)O[a][b]=R[a][0]*ORIGIN_ROT[j][0][b]+R[a][1]*ORIGIN_ROT[j][1][b]+R[a][2]*ORIGIN_ROT[j][2][b];
   p[a]+=R[a][0]*ORIGIN_XYZ[j][0]+R[a][1]*ORIGIN_XYZ[j][1]+R[a][2]*ORIGIN_XYZ[j][2];
  }
  for(int a=0;a<3;++a){R[a][0]=O[a][0]*c+O[a][1]*s;R[a][1]=-O[a][0]*s+O[a][1]*c;R[a][2]=O[a][2];}
 }
 for(int a=0;a<3;++a){
  for(int b=0;b<3;++b)T[a*4+b]=R[a][b];
  T[a*4+3]=p[a]+R[a][0]*ORIGIN_XYZ[6][0]+R[a][1]*ORIGIN_XYZ[6][1]+R[a][2]*ORIGIN_XYZ[6][2];
 }
}
extern "C" void native_fk_pose(const double *q,double *T){pose(q,T);}
extern "C" int native_fk_batch(const int32_t* co,const int32_t* box,unsigned samples,int32_t* out){
 std::fill(out,out+32,0);out[0]=0x464b4231;out[4]=-1;out[24]=MODEL_TAG;out[25]=1;
 bool valid=samples>=1 && samples<=4097;
 for(int a=0;a<3;++a){valid=valid && box[2*a]<=box[2*a+1];out[6+2*a]=INT32_MAX;out[7+2*a]=-INT32_MAX;}
 if(!valid){out[1]=1;return 1;}
 unsigned den=samples>1?samples-1:1;
 for(unsigned k=0;k<samples;++k){
  double u=double((uint64_t(k)<<24)/den)/16777216.,q[6],T[12];bool domain=true;
  for(int a=0;a<6;++a){double y=co[a*8+7]/16777216.;for(int b=6;b>=0;--b)y=y*u+co[a*8+b]/16777216.;q[a]=y;domain=domain && std::abs(y)<=6.283185307179586;}
  pose(q,T);unsigned mask=0;
  for(int a=0;a<3;++a){
   int32_t v=std::lrint(T[a*4+3]*16777216.);
   if(v<box[2*a])mask|=1u<<(2*a);
   if(v>box[2*a+1])mask|=1u<<(2*a+1);
   out[6+2*a]=std::min(out[6+2*a],v);out[7+2*a]=std::max(out[7+2*a],v);
  }
  if(!domain){out[1]|=2;mask|=64;}if(mask){++out[3];if(out[4]==-1)out[4]=k;out[5]|=mask;}++out[2];
  if(k==samples-1)for(int i=0;i<12;++i)out[12+i]=std::lrint(T[i]*16777216.);
 }
 return 0;
}
