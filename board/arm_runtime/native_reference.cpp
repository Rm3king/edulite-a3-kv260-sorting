// Optimized, bit-exact ARM alternative to the PL 21-channel degree-7 evaluator.
// No background thread, automatic enable/reset, or file/network access.
#include <cstdint>
#include <cstring>
#include <ctime>
#include <algorithm>
static inline int64_t round_even(int64_t v,unsigned n){
    int64_t q=v>>n,r=v-q*((int64_t)1<<n),h=(int64_t)1<<(n-1);
    return (int64_t)(q+(r>h || (r==h && (q&1))));
}
extern "C" int32_t native_horner(const int32_t* c,uint32_t phase){
    int64_t y=(int64_t)c[7]*65536;
    for(int k=6;k>=0;--k){
        // Exact decomposition avoids software 128-bit multiply on Cortex-A53.
        // Both products fit int64 for every possible int32 coefficient row.
        int64_t high=(y>>24)*phase,low=(y&16777215)*phase;
        int64_t floor=high+(low>>24),rem=low&16777215;
        y=floor+(rem>8388608 || (rem==8388608 && (floor&1)))+(int64_t)c[k]*65536;
    }
    return (int32_t)round_even(y,16);
}
extern "C" int native_fields(const int32_t* w,uint32_t tick,int32_t* out){
    const uint32_t n=(uint32_t)w[217];
    if(!n || tick>n)return -1;
    uint32_t phase=tick==n ? 16777216:(uint32_t)(((uint64_t)tick<<24)/n);
    for(int i=0;i<21;++i){
        out[i]=native_horner(w+8*i,phase);
        int f=i/7,a=i%7;
        if(out[i]<w[168+f*14+a] || out[i]>w[175+f*14+a])return -2;
    }
    return 0;
}
// Offline only: same integer core and every phase, one FFI call per segment.
// bounds are independent protocol-code [lo,hi] pairs for 3 fields x 6 axes.
extern "C" int native_validate_record(const int32_t* w,const int32_t* bounds,
        double claw_low,double claw_high,int rest,uint32_t* checked){
    *checked=0;
    if(w[217]<2 || w[217]>120000 || w[216]<0 || w[216]>512)return -5;
    const uint32_t n=(uint32_t)w[217];int32_t values[21];
    for(uint32_t tick=0;tick<=n;++tick){
        if(native_fields(w,tick,values))return -1;
        for(int f=0;f<3;++f)for(int a=0;a<6;++a){
            const int64_t code=((int64_t)values[7*f+a]*65535+8388608)>>24;
            const int b=2*(6*f+a);
            if(code<bounds[b] || code>bounds[b+1])return -2;
        }
        const double claw=(double)values[6]/16777216.;
        if(!(claw>=claw_low && claw<=claw_high))return -3;
        ++*checked;
    }
    if(rest)for(int a=0;a<6;++a)
        if((((int64_t)values[7+a]*65535+8388608)>>24)!=32768)return -4;
    return 0;
}
static inline uint32_t enc(int32_t q){return ((uint64_t)(uint32_t)q*65535+8388608)>>24;}
extern "C" int native_packets(const int32_t* w,uint32_t tick,uint32_t* packet){
    int32_t q[21];int rc=native_fields(w,tick,q);if(rc)return rc;
    for(int a=0;a<6;++a){
        packet[3*a]=0x1000000u+(enc(q[14+a])<<8)+(a+1);
        packet[3*a+1]=(enc(q[a])<<16)|enc(q[7+a]);
        packet[3*a+2]=(uint32_t)w[210+a];
    }
    float claw=(float)((double)q[6]/16777216.0);uint32_t bits;std::memcpy(&bits,&claw,4);
    packet[18]=0x1200fd07;packet[19]=0x16700000;
    packet[20]=__builtin_bswap32(bits);return 0;
}
static inline void barrier(){
#if defined(__aarch64__)
    asm volatile("dmb sy":::"memory");
#else
    __sync_synchronize();
#endif
}
static inline uint32_t rd(void* base,unsigned off){barrier();uint32_t v=*(volatile uint32_t*)((char*)base+off);barrier();return v;}
static inline void wr(void* base,unsigned off,uint32_t v){barrier();*(volatile uint32_t*)((char*)base+off)=v;barrier();}
// Precondition: the normal executor has configured all motor guards. Hardware
// repeats them. A preflight failure returns before committing any target.
extern "C" int native_submit(void* base,const int32_t* words,uint32_t tick,uint32_t due){
    uint32_t p[21];int r=native_packets(words,tick,p);if(r)return r;
    if((rd(base,0x404)&3)!=3 || rd(base,0x418) || rd(base,0x5a8)!=127)return -3;
    if((rd(base,0x41c)&65535)>9)return -4;
    if((int32_t)(due+4500-rd(base,0x408))<2000)return -5;
    for(int a=0;a<6;++a){
        unsigned b=0x600+16*a;
        uint32_t qb=rd(base,b),vb=rd(base,b+4),tb=rd(base,b+8),g=rd(base,b+12);
        uint32_t q=p[3*a+1]>>16,v=p[3*a+1]&65535,t=(p[3*a]>>8)&65535;
        if(q<(qb&65535)||q>(qb>>16)||v<(vb&65535)||v>(vb>>16)||t<(tb&65535)||t>(tb>>16)||
           (p[3*a+2]>>16)>(g&65535)||(p[3*a+2]&65535)>(g>>16))return -6;
    }
    const bool safe=rd(base,0x470)==0x60003;
    wr(base,0x410,1);
    for(int a=0;a<7;++a){
        wr(base,0x440,due);wr(base,0x444,due+4500);wr(base,0x448,p[3*a]);
        wr(base,0x44c,8);wr(base,0x450,p[3*a+1]);wr(base,0x454,p[3*a+2]);
        if(safe){
            uint32_t sequence=rd(base,0x478);wr(base,0x45c,1);
            if(rd(base,0x478)!=sequence+1 || rd(base,0x474))return -7;
        }else wr(base,0x458,1);
    }
    return 0;
}
extern "C" int native_upload(void* base,const int32_t* words){
    if((rd(base,0x404)&3)!=3 || rd(base,0x418) || rd(base,0xb44) || (rd(base,0xb40)&34))return -10;
    uint32_t before=rd(base,0xb60);
    for(unsigned i=0;i<220;++i)wr(base,0xc00+4*i,(uint32_t)words[i]);
    wr(base,0xb60,1);
    if(rd(base,0xb60)!=before || rd(base,0x418) || (rd(base,0x404)&3)!=3)return -11;
    return 0;
}
