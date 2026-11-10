// Native implementation of supplied 32-state full-car model.
// Preserves Ts=.01 RK4 substeps and observation timing. No fast-math.
// Unlike original, a nonfinite integrator result is a numerical failure,
// never silently replaced by zero. First-failure mode preserves stable masks.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <omp.h>
using State=std::array<double,32>;
constexpr double pi=3.141592653589793238462643383279502884;
constexpr double m=2218.042, ms=1967.686, Iz=4793., Ixx=924., Iyy=2500.;
constexpr double lf=1.472,lr=1.531,tf=1.633,tr=1.631,g=9.81,hr=.20,hcg=.541,Rw=.360,Iw=1.80;
const double xw[4]={lf,lf,-lr,-lr}, yw[4]={tf/2,-tf/2,tr/2,-tr/2};
const double MR[4]={.8613,.8613,.7970,.7970};
struct Params {
 double dm[4][17]{}; double ux,mu,sx,sy, delta[4],cs[4],sn[4],kw[4],fz0[4]; int wear[4];
 double time=5,Ts=.01; int sub=20, early=1;
 double vxlo=-10,vxhi=10,rlo=-.4,rhi=.4,plo=-.4,phi=.4; int nv=41,nr=31,np=31;
 Params(double speed,double mu_,double steer,const int *w): ux(speed/3.6),mu(mu_),sx(mu_/0.9),sy(mu_/0.9){
  for(int i=0;i<4;i++){delta[i]=(i<2?steer:0)*pi/180;cs[i]=cos(delta[i]);sn[i]=sin(delta[i]);kw[i]=30790*MR[i]*MR[i];wear[i]=w[i]<12.5?0:(w[i]<37.5?25:50);fz0[i]=m*g*(i<2?lr:lf)/(2*(lf+lr));}
 }
};
// Current=950 mA: interpolate the exact 900/1000-mA source rows once.
const double va[17]={-1048,-524,-393,-262,-131,-79,-52,-26,0,26,52,79,131,262,393,524,1048};
const double front9[17]={-4029,-3061,-2682,-2033,-1041,-599,-383,-164,0,136,297,460,852,1318,1505,1660,2171};
const double front10[17]={-4211,-3320,-2993,-2431,-1377,-903,-633,-306,0,200,429,612,1013,1408,1581,1728,2227};
const double rear9[17]={-4512,-3525,-3010,-2333,-1309,-743,-464,-208,0,169,340,521,874,1260,1434,1581,2105};
const double rear10[17]={-4802,-3801,-3406,-2789,-1688,-1106,-785,-398,0,247,485,681,998,1319,1493,1633,2140};
inline double damper(double v,int i,const Params&p){
 v=std::min(1048.,std::max(-1048.,v));int j=0;while(j<15 && v>va[j+1])j++;
 double beta=(v-va[j])/(va[j+1]-va[j]); return p.dm[i][j]+beta*(p.dm[i][j+1]-p.dm[i][j]);
}
inline void loads(const State&x,const Params&p,double *fz,double *el,double *raw){
 double kf=(p.kw[0]+p.kw[1])*tf*tf/4,kr=(p.kw[2]+p.kw[3])*tr*tr/4;
 double lam=kf/(kf+kr),missing=(m*hcg-ms*hr)*x[28];
 double shift[4]={-lam*missing/tf,lam*missing/tf,-(1-lam)*missing/tr,(1-lam)*missing/tr};
 for(int i=0;i<4;i++){double e=p.fz0[i]-200000*x[5+i];raw[i]=e+shift[i];
  el[i]=std::min(16000.,std::max(100.,std::isfinite(e)?e:p.fz0[i]));
  fz[i]=std::min(16000.,std::max(100.,std::isfinite(raw[i])?raw[i]:p.fz0[i]));}
}
inline void tire(double k,double a,double fz,int wear,const Params&p,double& fx,double& fy){
 double df=(fz-5425.)/5425.;double pd=1.145,pk1=24.125,pk2=15.;
 if(wear==25){pd*=.95;pk1*=.95;pk2*=.95;}else if(wear==50){pd*=.90;pk1*=.90;pk2*=.90;}
 double mux=pd-.2106*df,D=mux*fz*p.sx,C=1.647;
 double K=fz*(pk1+pk2*df)*exp(-.4832*df),B=fabs(C*D)<1e-9?0:K/(C*D);
 double E=std::min(1.,-.02192+.23613*df),v=B*(k-.001635+.0025297*df);
 fx=D*sin(C*atan(v-E*(v-atan(v))));
 double PD1,PD2,PE1,PE2,PK1,PK2,PH1,PH2,PV1,PV2;
 if(wear==0){C=1.5662;PD1=1.0278;PD2=-.17125;PE1=-1.1357;PE2=-1.0544;PK1=-25.85;PK2=2.1377;PH1=8.675e-4;PH2=7.046e-5;PV1=-.007239;PV2=-7.442e-5;}
 else if(wear==25){C=1.5457;PD1=1.0286;PD2=-.16688;PE1=-1.439;PE2=-1.6063;PK1=-26.;PK2=1.9164;PH1=1.82e-3;PH2=-7.193e-4;PV1=1.802e-3;PV2=-1.1497e-2;}
 else{C=1.5057;PD1=1.0067;PD2=-.2019;PE1=-2.;PE2=-2.;PK1=-27.27;PK2=1.5963;PH1=2.3735e-3;PH2=2.38e-4;PV1=.01487;PV2=-.011025;}
 D=(PD1+PD2*df)*fz*p.sy;K=PK1*5425.*sin(2*atan(fz/(PK2*5425.)));B=fabs(C*D)<1e-9?0:K/(C*D);
 E=std::min(1.,PE1+PE2*df);v=B*(a+PH1+PH2*df);
 fy=D*sin(C*atan(v-E*(v-atan(v))))+fz*(PV1+PV2*df);
 double cap=std::max(1.,std::max(.05,fabs(mux)*p.sx)*fz),rho=std::min(1.,fabs(fx)/cap);
 fy*=sqrt(std::max(0.,1-rho*rho));
}
inline void forces(const State&x,const Params&p,const double*fz,double*fxb,double*fyb,double*fxt){
 for(int i=0;i<4;i++){double fx=0,fy=0;if(fz[i]>100)tire(x[16+i],x[20+i],fz[i],p.wear[i],p,fx,fy);
  fxt[i]=fx;fxb[i]=fx*p.cs[i]-fy*p.sn[i];fyb[i]=fx*p.sn[i]+fy*p.cs[i];}
}
void rhs(const State&x,const Params&p,State&dx){
 double fz[4],el[4],raw[4],fxb[4],fyb[4],fxt[4],Fs[4];loads(x,p,fz,el,raw);forces(x,p,fz,fxb,fyb,fxt);
 double fy=0,mz=0,fs=0;
 dx.fill(0);
 for(int i=0;i<4;i++){
  double zB=x[2]+xw[i]*x[4]+yw[i]*x[3],vB=x[9]+xw[i]*x[11]+yw[i]*x[10];
  Fs[i]=-p.kw[i]*(zB-x[5+i])+MR[i]*damper(-1000*MR[i]*(vB-x[12+i]),i,p);
  fy+=fyb[i];mz+=xw[i]*fyb[i]-yw[i]*fxb[i];fs+=Fs[i];
  double ub=p.ux-x[1]*yw[i],vb=x[0]+x[1]*xw[i],uw=p.cs[i]*ub+p.sn[i]*vb,vw=-p.sn[i]*ub+p.cs[i]*vb;
  double us=uw;if(us>=0&&us<.1)us=.1;if(us<0&&us>-.1)us=-.1;
  double at=atan2(vw,us),kt=(Rw*x[24+i]-uw)/std::max(.5,fabs(uw));kt=std::min(.5,std::max(-.5,kt));
  double vr=std::max(.5,fabs(uw)),rk=std::min(500.,vr/.35),ra=std::min(500.,vr/.75);
  dx[5+i]=x[12+i];dx[12+i]=(-Fs[i]+el[i]-p.fz0[i])/((m-ms)/4);
  dx[16+i]=rk*(kt-x[16+i]);dx[20+i]=ra*(at-x[20+i]);dx[24+i]=-Rw*fxt[i]/Iw;
 }
 double ay=fy/m;
 dx[0]=ay-p.ux*x[1];dx[1]=mz/Iz;dx[2]=x[9];dx[3]=x[10];dx[4]=x[11];dx[9]=fs/ms;
 dx[10]=((tf/2)*(Fs[0]-Fs[1])+(tr/2)*(Fs[2]-Fs[3])+ms*hr*ay-ms*g*hr*x[3])/Ixx;
 dx[11]=(lf*(Fs[0]+Fs[1])-lr*(Fs[2]+Fs[3]))/Iyy;dx[28]=(ay-x[28])/.03;
 dx[29]=x[1];dx[30]=p.ux*cos(x[29])-x[0]*sin(x[29]);dx[31]=p.ux*sin(x[29])+x[0]*cos(x[29]);
}
