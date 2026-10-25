// PRIVATE quasisteady MF free-rolling comparison with dynamic roll reaction.
// Five body states; the hidden original210 tire transient subsystem is replaced
// by instantaneous exact wheel kinematics and the shifted zero-Fx MF root.
// This is neither the original full tire transient model nor paper-exact.
// The tire kernel, offsets, healthy coefficients and mu/.9 scaling are unchanged.
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <limits>
#include <omp.h>
constexpr double PI=3.141592653589793238462643383279502884;
constexpr double M=1300.,MS=1300.,IZZ=2500.,IXX=700.,LF=1.2,LR=1.2,L=2.4;
constexpr double DF=.7,DR=.7,H=.54,HPHI=.54,G=9.81,KPHI=43100.,CPHI=1400.;
// Baseline only: no acceleration/load fixed-point closure is needed.
static_assert(M*H-MS*HPHI==0.,"This experiment is restricted to the published baseline closure");
using State=std::array<double,5>;
struct Params {
 double ux,mu,sx,sy,delta,cs,sn,dt,time,bound; int nsteps;
 Params(double speed,double roadmu,double steer,double step=.01,double horizon=5):ux(speed/3.6),mu(roadmu),sx(roadmu/.9),sy(roadmu/.9),delta(steer*PI/180),cs(cos(delta)),sn(sin(delta)),dt(step),time(horizon){
  nsteps=std::max(1,int(std::ceil(time/dt)));dt=time/nsteps;
  double dfmax=std::max(fabs((100.-5425.)/5425.),fabs((16000.-5425.)/5425.));
  double fxmax=(1.145+.2106*dfmax)*16000*fabs(sx);
  double fymax=(1.0278+.17125*dfmax)*16000*fabs(sy)+16000*(.007239+7.442e-5*dfmax);
  bound=(2*fabs(sn)*fxmax+(2*fabs(cs)+2)*fymax)/M+1;
 }
};
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

struct Observation {bool valid=true;double ay=0,raw[4]{},fx[4]{},fy[4]{};};
Observation observe_rhs(const State&x,const Params&p,State*dx=nullptr){
 Observation o{};double nan=std::numeric_limits<double>::quiet_NaN();
 for(double v:x)if(!std::isfinite(v)){o.valid=false;o.ay=nan;for(int i=0;i<4;i++)o.raw[i]=o.fx[i]=o.fy[i]=nan;if(dx)dx->fill(nan);return o;}
 // Dynamic reaction extension, eliminated using unchanged paper roll Eq4:
 // D=(m*h-ms*hphi)*ay+Cphi*p+Kphi*phi+ms*g*hphi*(sin(phi)-phi).
 // Baseline m*h-ms*hphi=0 makes this explicit.
 double D=CPHI*x[4]+KPHI*x[3]+MS*G*HPHI*(sin(x[3])-x[3]);
 double front=D*LR/(2*DF*L),rear=D*LF/(2*DR*L);
 o.raw[0]=M*G*LR/(2*L)-front;o.raw[1]=M*G*LR/(2*L)+front;
 o.raw[2]=M*G*LF/(2*L)-rear;o.raw[3]=M*G*LF/(2*L)+rear;
 for(int i=0;i<4;i++){
  double z=std::min(16000.,std::max(100.,o.raw[i]));
  // Original wheel-frame geometry and low-speed alpha denominator guard.
  double yw=(i%2==0?1:-1)*(i<2?DF:DR),xx=i<2?LF:-LR;
  double ub=x[0]-x[2]*yw,vb=x[1]+x[2]*xx,c=i<2?p.cs:1,s=i<2?p.sn:0;
  double uw=c*ub+s*vb,vw=-s*ub+c*vb,us=uw;
  if(us>=0&&us<.1)us=.1;if(us<0&&us>-.1)us=-.1;
  double alpha=atan2(vw,us);
  // Source longitudinal MF has Vx=0. Exact free-rolling root is kappa=-Shx.
  // Do not set kappa=0: that would retain a nonzero shifted Fx.
  double kappa=.001635-.0025297*((z-5425.)/5425.);
  if(z>100)tire(kappa,alpha,z,0,p,o.fx[i],o.fy[i]);
 }
 o.ay=(p.sn*(o.fx[0]+o.fx[1])+p.cs*(o.fy[0]+o.fy[1])+o.fy[2]+o.fy[3])/M;
 if(dx){
  dx->fill(0);double mz=p.cs*DF*(o.fx[1]-o.fx[0])+DR*(o.fx[3]-o.fx[2])
   +p.sn*DF*(o.fy[0]-o.fy[1])+p.sn*LF*(o.fx[0]+o.fx[1])
   +p.cs*LF*(o.fy[0]+o.fy[1])-LR*(o.fy[2]+o.fy[3]);
  (*dx)[1]=o.ay-x[0]*x[2];(*dx)[2]=mz/IZZ;(*dx)[3]=x[4];
  (*dx)[4]=(MS*HPHI*o.ay-CPHI*x[4]+(MS*G*HPHI-KPHI)*x[3])/IXX;

 }
 return o;
}
void step(State&x,const Params&p){
 constexpr int SUB=20;double h=p.dt/SUB;State k1{},k2{},k3{},k4{},z{};
 for(int sub=0;sub<SUB;sub++){
  observe_rhs(x,p,&k1);for(int i=0;i<5;i++)z[i]=x[i]+.5*h*k1[i];
  observe_rhs(z,p,&k2);for(int i=0;i<5;i++)z[i]=x[i]+.5*h*k2[i];
  observe_rhs(z,p,&k3);for(int i=0;i<5;i++)z[i]=x[i]+h*k3[i];
  observe_rhs(z,p,&k4);for(int i=0;i<5;i++)x[i]+=h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
 }

}
State initial(double v,double r,double phi,const Params&p){
 State x{};x[0]=p.ux;x[1]=v;x[2]=r;x[3]=phi;

 return x;
}
struct Result {int reason=0,observations=0;double maxyaw=0,maxay=0,minload=INFINITY;State x{};};
Result trajectory(double v,double r,double phi,const Params&p){
 Result z;z.x=initial(v,r,phi,p);double rlim=.85*p.mu*G/p.ux;
 for(int k=0;k<=p.nsteps;k++){
  auto o=observe_rhs(z.x,p);z.observations++;
  bool finite=o.valid&&std::isfinite(o.ay);
  if(fabs(z.x[2])>rlim+1e-10)z.reason|=1;
  if(fabs(o.ay)>p.mu*G+1e-10)z.reason|=2;
  for(double q:o.raw){if(q<=0)z.reason|=4;finite=finite&&std::isfinite(q);z.minload=std::min(z.minload,q);}
  if(!finite||z.x[0]<.5)z.reason|=8;
  z.maxyaw=std::max(z.maxyaw,fabs(z.x[2]));z.maxay=std::max(z.maxay,fabs(o.ay));
  if(z.reason||k==p.nsteps)break;step(z.x,p);
 }
 return z;
}
int main(int argc,char**argv){
 if(argc<5){std::cerr<<"Modes: rhs|step speed mu steer (5-state stdin); points speed mu steer input output [Ts time workers]\n";return 2;}
 std::string mode=argv[1];double speed=std::stod(argv[2]),mu=std::stod(argv[3]),steer=std::stod(argv[4]);Params p(speed,mu,steer);
 std::cout<<std::setprecision(17);
 if(mode=="rhs"||mode=="step"){
  State x{};while(std::cin>>x[0]){
   for(int i=1;i<5;i++)if(!(std::cin>>x[i]))return 3;
   if(mode=="step"){step(x,p);for(double v:x)std::cout<<v<<" ";std::cout<<"\n";}
   else{State d{};auto o=observe_rhs(x,p,&d);for(double v:d)std::cout<<v<<" ";std::cout<<o.ay<<" ";for(double v:o.raw)std::cout<<v<<" ";for(double v:o.fx)std::cout<<v<<" ";for(double v:o.fy)std::cout<<v<<" ";std::cout<<"\n";}
  }return 0;
 }
 if(mode!="points"||argc<7)return 2;double dt=argc>7?std::stod(argv[7]):.01,time=argc>8?std::stod(argv[8]):5;
 int workers=argc>9?std::stoi(argv[9]):2;p=Params(speed,mu,steer,dt,time);omp_set_num_threads(workers);
 std::ifstream in(argv[5]);if(!in)return 3;std::vector<std::array<double,3>>points;std::array<double,3>q{};
 while(in>>q[0]>>q[1]>>q[2])points.push_back(q);if(points.empty())return 3;
 std::vector<Result>out(points.size());std::atomic<int>completed{0},stable{0};int lastwritten=0;
 auto start=std::chrono::steady_clock::now();std::string progress=std::string(argv[6])+".progress.json";
 #pragma omp parallel for schedule(dynamic,16)
 for(size_t i=0;i<points.size();i++){
  out[i]=trajectory(points[i][0],points[i][1],points[i][2],p);if(out[i].reason==0)stable++;int c=++completed;
  if(c%128==0||c==int(points.size())){
   #pragma omp critical(progress_write)
   {if(c>lastwritten){lastwritten=c;double e=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();std::string tmp=progress+".tmp";std::ofstream f(tmp);f<<"{\"processed_initial_states\":"<<c<<",\"total_initial_states\":"<<points.size()<<",\"stable_so_far\":"<<stable.load()<<",\"elapsed_s\":"<<e<<"}\n";f.close();std::rename(tmp.c_str(),progress.c_str());}}
  }
 }
 std::ofstream f(argv[6]);if(!f)return 4;f<<std::setprecision(17);
 f<<"reason,max_abs_yaw,max_abs_ay,min_wheel_load,max_load_residual,max_solver_iterations,solver_failure,observations,u,vc,r,phi,p\n";
 for(const auto&z:out){f<<z.reason<<","<<z.maxyaw<<","<<z.maxay<<","<<z.minload<<",0,0,0,"<<z.observations;for(double v:z.x)f<<","<<v;f<<"\n";}
 double e=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();std::cerr<<"MF QUASISTEADY FREE-ROLLING COMPARISON COMPLETED "<<points.size()<<" stable "<<stable<<" seconds "<<e<<"\n";return 0;
}

