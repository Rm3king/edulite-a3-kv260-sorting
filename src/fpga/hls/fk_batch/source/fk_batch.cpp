#include "fk_batch.hpp"
#include "model_constants.hpp"
// Valid rotation/position intermediates fit (-8,8); q domain is checked.
// Horner with 8 coefficients in [-128,128) and u in [0,1] fits (-2048,2048).
typedef ap_fixed<32,4,AP_RND_CONV,AP_WRAP> value_t;
typedef ap_fixed<40,12,AP_RND_CONV,AP_WRAP> poly_t;
typedef ap_ufixed<25,1> phase_t;
static value_t bits_value(int bits) {
#pragma HLS INLINE
 value_t v;v.range(31,0)=ap_int<32>(bits);return v;
}
static int q24(value_t value) {
#pragma HLS INLINE
 ap_fixed<32,8,AP_RND_CONV,AP_WRAP> v=value;return v.range(31,0).to_int();
}
template<int ID> static void trig(value_t q,value_t &s,value_t &c) {
#pragma HLS INLINE off
#pragma HLS PIPELINE off
 int z=q.range(31,0).to_int();int flip=1;
 if(z>843314857)z-=1686629713;
 if(z< -843314857)z+=1686629713;
 if(z>421657428){z-=843314857;flip=-1;}
 if(z< -421657428){z+=843314857;flip=-1;}
 int x=163008219,y=0;
 for(int k=0;k<28;++k){
#pragma HLS PIPELINE II=1
  int xn,yn;
  if(z>=0){xn=x-(y>>k);yn=y+(x>>k);z-=CORDIC_ATAN[k];}
  else{xn=x+(y>>k);yn=y-(x>>k);z+=CORDIC_ATAN[k];}
  x=xn;y=yn;
 }
 c=bits_value(flip<0?-x:x);s=bits_value(flip<0?-y:y);
}
static void pose(value_t q[6],value_t T[3][4]) {
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=q complete
#pragma HLS ARRAY_PARTITION variable=T complete dim=0
 value_t ss[6],cc[6];
#pragma HLS ARRAY_PARTITION variable=ss complete
#pragma HLS ARRAY_PARTITION variable=cc complete
 trig<0>(q[0],ss[0],cc[0]);trig<1>(q[1],ss[1],cc[1]);trig<2>(q[2],ss[2],cc[2]);
 trig<3>(q[3],ss[3],cc[3]);trig<4>(q[4],ss[4],cc[4]);trig<5>(q[5],ss[5],cc[5]);
 value_t R[7][3][3],p[7][3];
#pragma HLS ARRAY_PARTITION variable=R complete dim=0
#pragma HLS ARRAY_PARTITION variable=p complete dim=0
 for(int a=0;a<3;++a){
#pragma HLS UNROLL
  p[0][a]=0;
  for(int b=0;b<3;++b){
#pragma HLS UNROLL
   R[0][a][b]=(a==b)?1:0;
  }
 }
 for(int j=0;j<6;++j){
#pragma HLS UNROLL
  value_t s=ss[j],c=cc[j],O[3][3];
#pragma HLS ARRAY_PARTITION variable=O complete dim=0
  for(int a=0;a<3;++a){
#pragma HLS UNROLL
   p[j+1][a]=p[j][a]+R[j][a][0]*value_t(ORIGIN_XYZ[j][0])+R[j][a][1]*value_t(ORIGIN_XYZ[j][1])+R[j][a][2]*value_t(ORIGIN_XYZ[j][2]);
   for(int b=0;b<3;++b){
#pragma HLS UNROLL
    O[a][b]=R[j][a][0]*value_t(ORIGIN_ROT[j][0][b])+R[j][a][1]*value_t(ORIGIN_ROT[j][1][b])+R[j][a][2]*value_t(ORIGIN_ROT[j][2][b]);
   }
   R[j+1][a][0]=O[a][0]*c+O[a][1]*s;
   R[j+1][a][1]=-O[a][0]*s+O[a][1]*c;
   R[j+1][a][2]=O[a][2];
  }
 }
 for(int a=0;a<3;++a){
#pragma HLS UNROLL
  for(int b=0;b<3;++b){
#pragma HLS UNROLL
   T[a][b]=R[6][a][b];
  }
  T[a][3]=p[6][a]+R[6][a][0]*value_t(ORIGIN_XYZ[6][0])+R[6][a][1]*value_t(ORIGIN_XYZ[6][1])+R[6][a][2]*value_t(ORIGIN_XYZ[6][2]);
 }
}
void fk_batch(const int coefficient[48], const int box[6], unsigned samples,int result[32]) {
#pragma HLS INTERFACE s_axilite port=return bundle=control
#pragma HLS INTERFACE s_axilite port=samples bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=coefficient bundle=control offset=0x100
#pragma HLS INTERFACE s_axilite port=box bundle=control offset=0x200
#pragma HLS INTERFACE s_axilite port=result bundle=control offset=0x300
 int words[48],bounds[6],out[32];
#pragma HLS ARRAY_PARTITION variable=words complete
#pragma HLS ARRAY_PARTITION variable=bounds complete
#pragma HLS ARRAY_PARTITION variable=out complete
 for(int i=0;i<48;++i)words[i]=coefficient[i];
 for(int i=0;i<6;++i)bounds[i]=box[i];
 for(int i=0;i<32;++i)out[i]=0;
 out[0]=0x464b4231;out[4]=-1;out[24]=MODEL_TAG;out[25]=1;
 bool valid=samples>=1 && samples<=4097;
 for(int i=0;i<3;++i){
#pragma HLS UNROLL
  valid=valid && bounds[2*i]<=bounds[2*i+1];out[6+2*i]=2147483647;out[7+2*i]=-2147483647;
 }
 if(!valid){out[1]=1;for(int i=0;i<32;++i)result[i]=out[i];return;}
 unsigned den=samples>1?samples-1:1,step=(1u<<24)/den,rem=(1u<<24)%den,phase=0,carry=0;
 for(unsigned k=0;k<samples;++k){
#pragma HLS LOOP_TRIPCOUNT min=1 max=4097 avg=51
#pragma HLS PIPELINE II=32
  value_t q[6],T[3][4];bool domain=true;
#pragma HLS ARRAY_PARTITION variable=q complete
#pragma HLS ARRAY_PARTITION variable=T complete dim=0
  phase_t u;u.range(24,0)=phase;
  for(int a=0;a<6;++a){
#pragma HLS UNROLL
   ap_fixed<32,8> v;v.range(31,0)=ap_int<32>(words[a*8+7]);poly_t y=v;
   for(int b=6;b>=0;--b){
#pragma HLS UNROLL
    v.range(31,0)=ap_int<32>(words[a*8+b]);y=y*u+v;
   }
   domain=domain && y>=poly_t(-6.283185307179586) && y<=poly_t(6.283185307179586);q[a]=y;
  }
  pose(q,T);
  unsigned mask=0;
  for(int a=0;a<3;++a){
#pragma HLS UNROLL
   int x=q24(T[a][3]);
   if(x<bounds[2*a])mask|=1u<<(2*a);
   if(x>bounds[2*a+1])mask|=1u<<(2*a+1);
   if(x<out[6+2*a])out[6+2*a]=x;
   if(x>out[7+2*a])out[7+2*a]=x;
  }
  if(!domain){out[1]|=2;mask|=64;}
  if(mask){++out[3];if(out[4]==-1)out[4]=k;out[5]|=mask;}
  ++out[2];
  if(k==samples-1){
   for(int a=0;a<3;++a){
#pragma HLS UNROLL
    for(int b=0;b<4;++b){
#pragma HLS UNROLL
     out[12+a*4+b]=q24(T[a][b]);
    }
   }
  }
  phase+=step;carry+=rem;if(carry>=den){carry-=den;++phase;}
 }
 for(int i=0;i<32;++i)result[i]=out[i];
}
