// Literal paper Eq11 baseline; original MF force law; implicit normal loads.
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
// Baseline mass/height assumptions retained; Eq11 now has an implicit ay loop.
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

struct WheelData {double alpha[4]{},roll=0;};
struct Observation {
 bool valid=true,ambiguous=false,fallback=false;
 double ay=0,root=0,residual=0,loadResidual=0,raw[4]{},fx[4]{},fy[4]{};
 int evaluations=0,candidates=1;
};
constexpr double ROOT_TOL=2e-10;
WheelData wheel_data(const State&x,const Params&p){
 WheelData w;w.roll=MS*G*HPHI*sin(x[3]);
 for(int i=0;i<4;i++){
  double yw=(i%2==0?1:-1)*(i<2?DF:DR),xx=i<2?LF:-LR;
  double ub=x[0]-x[2]*yw,vb=x[1]+x[2]*xx,c=i<2?p.cs:1,s=i<2?p.sn:0;
  double uw=c*ub+s*vb,vw=-s*ub+c*vb,us=uw;
  if(us>=0&&us<.1)us=.1;if(us<0&&us>-.1)us=-.1;
  w.alpha[i]=atan2(vw,us);
 }
 return w;
}
Observation evaluate_loads(double a,const WheelData&w,const Params&p){
 Observation o;o.root=a;double D=M*H*a+w.roll;
 double front=D*LR/(2*DF*L),rear=D*LF/(2*DR*L);
 o.raw[0]=M*G*LR/(2*L)-front;o.raw[1]=M*G*LR/(2*L)+front;
 o.raw[2]=M*G*LF/(2*L)-rear;o.raw[3]=M*G*LF/(2*L)+rear;
 for(int i=0;i<4;i++){
  double z=std::min(16000.,std::max(100.,o.raw[i]));
  double kappa=.001635-.0025297*((z-5425.)/5425.);
  if(z>100)tire(kappa,w.alpha[i],z,0,p,o.fx[i],o.fy[i]);
 }
 o.ay=(p.sn*(o.fx[0]+o.fx[1])+p.cs*(o.fy[0]+o.fy[1])+o.fy[2]+o.fy[3])/M;
 o.residual=fabs(a-o.ay);
 o.loadResidual=o.residual*std::max(M*H*LR/(2*DF*L),M*H*LF/(2*DR*L));
 return o;
}
Observation solve_loads(const WheelData&w,const Params&p,double hint=NAN){
 int neval=0;auto eval=[&](double a){neval++;return evaluate_loads(a,w,p);};
 auto finish=[&](Observation o,bool fallback,int candidates){o.evaluations=neval;o.fallback=fallback;o.candidates=candidates;return o;};
 double a0=std::isfinite(hint)?std::clamp(hint,-p.bound,p.bound):0;
 Observation o0=eval(a0);double f0=a0-o0.ay;
 if(o0.residual<=ROOT_TOL)return finish(o0,false,1);
 double a1=std::clamp(o0.ay,-p.bound,p.bound);
 for(int iteration=0;iteration<10;iteration++){
  Observation o1=eval(a1);double f1=a1-o1.ay;
  if(o1.residual<=ROOT_TOL)return finish(o1,false,1);
  double den=f1-f0;
  if(!std::isfinite(den)||fabs(den)<1e-16||!std::isfinite(f1))break;
  double next=a1-f1*(a1-a0)/den;
  if(!std::isfinite(next)||fabs(next)>p.bound||fabs(next-a1)<1e-14)break;
  a0=a1;f0=f1;a1=next;
 }
 // Activation at raw Fz=100 N is discontinuous in the inherited force
 // law. Split there (also at the 16000 N clipping threshold); never accept
 // a mere bracket collapse without certifying the actual force residual.
 std::vector<double> cuts={-p.bound,p.bound};
 double aa[4],bb[4];
 for(int i=0;i<4;i++){
  double sign=i%2==0?-1:1,share=i<2?LR:LF,d=i<2?DF:DR;
  aa[i]=M*G*share/(2*L)+sign*w.roll*share/(2*d*L);
  bb[i]=sign*M*H*share/(2*d*L);
  for(double load:{100.,16000.}){double a=(load-aa[i])/bb[i];if(a>-p.bound&&a<p.bound)cuts.push_back(a);}
 }
 std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
 std::vector<Observation> roots;
 auto append=[&](Observation o){
  if(!std::isfinite(o.ay)||o.residual>ROOT_TOL)return;
  for(const auto&r:roots)if(fabs(r.root-o.root)<1e-7)return;
  roots.push_back(o);
 };
 for(double c:cuts)append(eval(c));
 for(size_t seg=0;seg+1<cuts.size();seg++){
  double left=cuts[seg],right=cuts[seg+1];
  double eps=32*std::numeric_limits<double>::epsilon()*std::max({1.,fabs(left),fabs(right)});
  left+=eps;right-=eps;if(left>=right)continue;
  Observation prev=eval(left);append(prev);double pa=left,pf=left-prev.ay;
  for(int sample=1;sample<=16;sample++){
   double ca=left+(right-left)*sample/16.;Observation cur=eval(ca);append(cur);double cf=ca-cur.ay;
   if(pf*cf<0){
    double lo=pa,hi=ca,fl=pf,fh=cf;
    for(int iteration=0;iteration<80;iteration++){
     double trial=(lo*fh-hi*fl)/(fh-fl),width=hi-lo;
     if(!std::isfinite(trial)||trial<=lo+.05*width||trial>=hi-.05*width)trial=(lo+hi)/2;
     Observation q=eval(trial);double fq=trial-q.ay;
     if(q.residual<=ROOT_TOL){append(q);break;}
     if(fl*fq<0){hi=trial;fh=fq;}else{lo=trial;fl=fq;}
     if(hi-lo<1e-14)break;
    }
   }
   pa=ca;pf=cf;
  }
 }
 std::vector<int> loaded;
 for(size_t i=0;i<roots.size();i++){bool full=true;for(double z:roots[i].raw)full=full&&z>0;if(full)loaded.push_back(i);}
 if(loaded.size()==1)return finish(roots[loaded[0]],true,roots.size());
 if(loaded.empty()&&roots.size()==1)return finish(roots[0],true,1);
 Observation bad;bad.valid=false;bad.ambiguous=roots.size()>1;bad.candidates=roots.size();bad.ay=NAN;
 bad.residual=INFINITY;bad.loadResidual=INFINITY;
 for(int i=0;i<4;i++)bad.raw[i]=bad.fx[i]=bad.fy[i]=NAN;
 return finish(bad,true,roots.size());
}
struct Stats {
 long long calls=0,evaluations=0,fallbacks=0,ambiguities=0;
 int maxEvaluations=0,maxCandidates=0;double maxResidual=0,maxLoadResidual=0;
 void add(const Observation&o){calls++;evaluations+=o.evaluations;fallbacks+=o.fallback;ambiguities+=o.ambiguous;
  maxEvaluations=std::max(maxEvaluations,o.evaluations);maxCandidates=std::max(maxCandidates,o.candidates);
  maxResidual=std::max(maxResidual,o.residual);maxLoadResidual=std::max(maxLoadResidual,o.loadResidual);}
};
Observation observe_rhs(const State&x,const Params&p,State*dx=nullptr,double hint=NAN,Stats*stats=nullptr){
 for(double v:x)if(!std::isfinite(v)){
  Observation o;o.valid=false;o.ay=NAN;o.residual=o.loadResidual=INFINITY;
  for(int i=0;i<4;i++)o.raw[i]=o.fx[i]=o.fy[i]=NAN;
  if(dx)dx->fill(NAN);if(stats)stats->add(o);return o;
 }
 Observation o=solve_loads(wheel_data(x,p),p,hint);if(stats)stats->add(o);
 if(dx){
  if(!o.valid){dx->fill(NAN);return o;}
  dx->fill(0);double mz=p.cs*DF*(o.fx[1]-o.fx[0])+DR*(o.fx[3]-o.fx[2])
   +p.sn*DF*(o.fy[0]-o.fy[1])+p.sn*LF*(o.fx[0]+o.fx[1])
   +p.cs*LF*(o.fy[0]+o.fy[1])-LR*(o.fy[2]+o.fy[3]);
  (*dx)[1]=o.ay-x[0]*x[2];(*dx)[2]=mz/IZZ;(*dx)[3]=x[4];
  (*dx)[4]=(MS*HPHI*o.ay-CPHI*x[4]+(MS*G*HPHI-KPHI)*x[3])/IXX;
 }
 return o;
}
bool step(State&x,const Params&p,double&hint,Stats*stats=nullptr){
 constexpr int SUB=20;double h=p.dt/SUB;State k1{},k2{},k3{},k4{},z{};
 auto rhs=[&](const State&q,State&k){auto o=observe_rhs(q,p,&k,hint,stats);if(o.valid)hint=o.root;return o.valid;};
 for(int sub=0;sub<SUB;sub++){
  if(!rhs(x,k1))return false;for(int i=0;i<5;i++)z[i]=x[i]+.5*h*k1[i];
  if(!rhs(z,k2))return false;for(int i=0;i<5;i++)z[i]=x[i]+.5*h*k2[i];
  if(!rhs(z,k3))return false;for(int i=0;i<5;i++)z[i]=x[i]+h*k3[i];
  if(!rhs(z,k4))return false;for(int i=0;i<5;i++)x[i]+=h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
 }
 return true;
}
struct Result {int reason=0,observations=0,solverFailure=0;double maxyaw=0,maxay=0,minload=INFINITY;State x{};Stats stats;};
Result trajectory(double v,double r,double phi,const Params&p){
 Result z;z.x={p.ux,v,r,phi,0};double rlim=.85*p.mu*G/p.ux,hint=NAN;
 for(int k=0;k<=p.nsteps;k++){
  auto o=observe_rhs(z.x,p,nullptr,hint,&z.stats);z.observations++;
  if(!o.valid||!std::isfinite(o.ay)||z.x[0]<.5){z.reason|=8;z.solverFailure=1;break;}
  hint=o.root;
  if(fabs(z.x[2])>rlim+1e-10)z.reason|=1;
  if(fabs(o.ay)>p.mu*G+1e-10)z.reason|=2;
  for(double q:o.raw){if(q<=0)z.reason|=4;z.minload=std::min(z.minload,q);}
  z.maxyaw=std::max(z.maxyaw,fabs(z.x[2]));z.maxay=std::max(z.maxay,fabs(o.ay));
  if(z.reason||k==p.nsteps)break;
  if(!step(z.x,p,hint,&z.stats)){z.reason|=8;z.solverFailure=1;break;}
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
   if(mode=="step"){double hint=NAN;if(!step(x,p,hint))x.fill(NAN);for(double v:x)std::cout<<v<<" ";std::cout<<"\n";}
   else{State d{};auto o=observe_rhs(x,p,&d);for(double v:d)std::cout<<v<<" ";std::cout<<o.ay<<" ";
    for(double v:o.raw)std::cout<<v<<" ";for(double v:o.fx)std::cout<<v<<" ";for(double v:o.fy)std::cout<<v<<" ";
    std::cout<<o.residual<<" "<<o.loadResidual<<" "<<o.evaluations<<" "<<o.fallback<<" "<<o.candidates<<" "<<o.valid<<" "<<o.ambiguous<<"\n";}
  }return 0;
 }
 if(mode!="points"||argc<7)return 2;double dt=argc>7?std::stod(argv[7]):.01,time=argc>8?std::stod(argv[8]):5;
 int workers=argc>9?std::stoi(argv[9]):2;p=Params(speed,mu,steer,dt,time);omp_set_num_threads(workers);
 std::ifstream in(argv[5]);if(!in)return 3;std::vector<std::array<double,3>>points;std::array<double,3>q{};
 while(in>>q[0]>>q[1]>>q[2])points.push_back(q);if(points.empty())return 3;
 std::vector<Result>out(points.size());std::atomic<int>completed{0},stable{0},numerical{0};int lastwritten=0;
 auto start=std::chrono::steady_clock::now();std::string progress=std::string(argv[6])+".progress.json";
 #pragma omp parallel for schedule(dynamic,16)
 for(size_t i=0;i<points.size();i++){
  out[i]=trajectory(points[i][0],points[i][1],points[i][2],p);if(out[i].reason==0)stable++;if(out[i].solverFailure)numerical++;int c=++completed;
  if(c%128==0||c==int(points.size())){
   #pragma omp critical(progress_write)
   {if(c>lastwritten){lastwritten=c;double e=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();std::string tmp=progress+".tmp";std::ofstream f(tmp);f<<"{\"processed_initial_states\":"<<c<<",\"total_initial_states\":"<<points.size()<<",\"stable_so_far\":"<<stable.load()<<",\"numerical_failures\":"<<numerical.load()<<",\"elapsed_s\":"<<e<<"}\n";f.close();std::rename(tmp.c_str(),progress.c_str());}}
  }
 }
 std::ofstream f(argv[6]);if(!f)return 4;f<<std::setprecision(17);
 f<<"reason,max_abs_yaw,max_abs_ay,min_wheel_load,max_load_residual,max_solver_iterations,solver_failure,observations,u,vc,r,phi,p,closure_calls,closure_force_evaluations,closure_fallback_calls,max_closure_candidates,closure_ambiguity_count,max_closure_residual_m_s2\n";
 for(const auto&z:out){f<<z.reason<<","<<z.maxyaw<<","<<z.maxay<<","<<z.minload<<","<<z.stats.maxLoadResidual<<","<<z.stats.maxEvaluations<<","<<z.solverFailure<<","<<z.observations;for(double v:z.x)f<<","<<v;
  f<<","<<z.stats.calls<<","<<z.stats.evaluations<<","<<z.stats.fallbacks<<","<<z.stats.maxCandidates<<","<<z.stats.ambiguities<<","<<z.stats.maxResidual<<"\n";}
 double e=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();std::cerr<<"MF LITERAL EQ11 COMPLETED "<<points.size()<<" stable "<<stable<<" numerical "<<numerical<<" seconds "<<e<<"\n";return 0;
}
