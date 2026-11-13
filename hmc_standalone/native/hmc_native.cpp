#include "hmc_vehicle.hpp"
#include <iostream>
#include <sstream>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace {
using hmc::Json;using hmc::State;
constexpr int NF=23;
struct Policy {
 double speed,mu,steer,ltr_limit,roll_limit,yaw_limit,ay_limit,window,dt;
 bool allow_trim_outside;std::vector<double>horizons;std::array<int,4>wear;
 std::array<double,7>tol;
 Policy(const Json&config){const auto&c=config.at("c");speed=c.num("speed_kmh",3.6*c.num("ux",50/3.6));mu=c.num("mu",.5);steer=c.num("reference_steer_deg",c.num("steer_actual_deg",c.num("steer_deg",0)))*hmc::rad;dt=c.num("dt",.0005);
  ltr_limit=c.num("LTRLimit",1);roll_limit=c.num("rollAbsoluteLimit_deg",40)*hmc::rad;yaw_limit=c.num("yawPolicyFactor",.85)*mu*9.81/(speed/3.6);ay_limit=c.num("ayPolicyFactor",1)*mu*9.81;window=c.num("lastWindow_s",c.num("terminal_window",1));allow_trim_outside=c.num("allowTrimOutsideH",0)!=0;
  horizons=c.has("horizons_s")?c.vec("horizons_s"):(c.has("horizons")?c.vec("horizons"):std::vector<double>{5,10});
  auto w=config.at("wear").vec();if(w.size()!=4)throw std::runtime_error("Wear must have four entries");for(int i=0;i<4;++i)wear[i]=1+(w[i]>=12.5)+(w[i]>=37.5);
  tol={.10,.005,.2*hmc::rad,.5*hmc::rad,.2*hmc::rad,.5*hmc::rad,.5/3.6};
  if(c.has("terminalTol")){const auto&t=c.at("terminalTol");tol={t.num("vy_mps",.10),t.num("r_radps",.005),t.num("roll_deg",.2)*hmc::rad,t.num("rollrate_degps",.5)*hmc::rad,t.num("pitch_deg",.2)*hmc::rad,t.num("pitchrate_degps",.5)*hmc::rad,t.num("vx_kmh",.5)/3.6};}
  if(!(dt>0&&window>0&&speed>0&&mu>0)||horizons.empty())throw std::runtime_error("Invalid time/policy configuration");
  for(std::size_t i=0;i<horizons.size();++i)if(horizons[i]<window||(i&&horizons[i]<=horizons[i-1])||std::abs(horizons[i]/dt-std::round(horizons[i]/dt))>1e-7)throw std::runtime_error("Horizon not on fixed-step lattice");
 }
 bool handling(double vy,double r)const{
  static constexpr double M[3][12]={{1.5662,1.0278,-.17125,-1.1357,-1.0544,-25.85,2.1377,2,8.675e-4,7.046e-5,-.007239,-7.442e-5},{1.5457,1.0286,-.16688,-1.439,-1.6063,-26,1.9164,2,1.82e-3,-7.193e-4,1.802e-3,-1.1497e-2},{1.5057,1.0067,-.2019,-2,-2,-27.27,1.5963,2,2.3735e-3,2.38e-4,.01487,-.011025}};
  constexpr double mass=2218.042,Iz=4793,lf=1.472,lr=1.531,tf=1.633,tr=1.631;double u=speed/3.6;
  const double x[4]={lf,lf,-lr,-lr},y[4]={tf/2,-tf/2,tr/2,-tr/2};
  double a11=0,a12=-u,a21=0,a22=0;
  for(int i=0;i<4;++i){double fz=mass*9.81*(i<2?lr:lf)/(2*(lf+lr)),df=(fz-5425)/5425;const double*m=M[wear[i]-1];double d=i<2?steer:0,cd=std::cos(d),sd=std::sin(d),ub=u-r*y[i],vb=vy+r*x[i],uw=cd*ub+sd*vb;
   if(!(uw>.1))return false;double alpha=std::atan2(-sd*ub+cd*vb,uw),D=(m[1]+m[2]*df)*fz*mu/.9,K=m[5]*5425*std::sin(m[7]*std::atan(fz/(m[6]*5425))),B=std::abs(m[0]*D)<1e-9?0:K/(m[0]*D),E=std::min(m[3]+m[4]*df,1.0),z=B*(alpha+m[8]+m[9]*df),q=z-E*(z-std::atan(z)),ang=m[0]*std::atan(q),der=D*std::cos(ang)*m[0]/(1+q*q)*B*(1-E+E/(1+z*z));
   if(!std::isfinite(der)||!(-der>0))return false;double den=ub*ub+vb*vb,av=ub/den,ar=(x[i]*ub+y[i]*vb)/den,b=cd/mass,zc=(x[i]*cd+y[i]*sd)/Iz;a11+=b*der*av;a12+=b*der*ar;a21+=zc*der*av;a22+=zc*der*ar;
  }double trc=a11+a22,det=a11*a22-a12*a21;return std::isfinite(trc)&&std::isfinite(det)&&trc<0&&det>0;
 }
};
State trim_state(const Json&t){State x{};if(t.has("x")){auto a=t.vec("x");if(a.size()!=34)throw std::runtime_error("Trim must have34 states");std::copy(a.begin(),a.end(),x.begin());}else{x[0]=t.num("vy_mps");x[1]=t.num("r_radps");x[3]=t.num("roll_deg")*hmc::rad;x[4]=t.num("pitch_deg")*hmc::rad;x[10]=t.num("rollrate_degps")*hmc::rad;x[11]=t.num("pitchrate_degps")*hmc::rad;x[32]=t.num("vx_kmh")/3.6;}return x;}
State rk4(const State&x,double h,const hmc::Vehicle&m){State k1,k2,k3,k4,z,y;m.rhs(x,k1,nullptr);for(int i=0;i<34;++i)z[i]=x[i]+.5*h*k1[i];m.rhs(z,k2,nullptr);for(int i=0;i<34;++i)z[i]=x[i]+.5*h*k2[i];m.rhs(z,k3,nullptr);for(int i=0;i<34;++i)z[i]=x[i]+h*k3[i];m.rhs(z,k4,nullptr);for(int i=0;i<34;++i)y[i]=x[i]+h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;return y;}
bool finite(const State&x){for(double a:x)if(!std::isfinite(a)||std::abs(a)>1e10)return false;return true;}
struct History{
 bool physical=true,inside=true,yaw=true,ay=true;double firstH=NAN,firstPhysical=NAN,firstAy=NAN,firstYaw=NAN,minFz=INFINITY,maxLtr=0,maxLtrF=0,maxLtrR=0,peakP=0,peakPhi=0,peakR=0,peakAy=0,initialP=0,lastP=0,observed=0;
 void update(double t,const State&x,const hmc::Observation&o,const Policy&p){
  double fl=o.fz[0],fr=o.fz[1],rl=o.fz[2],rr=o.fz[3];double lt=(fl+rl-fr-rr)/std::max(fl+fr+rl+rr,std::numeric_limits<double>::epsilon()),lf=(fl-fr)/std::max(fl+fr,std::numeric_limits<double>::epsilon()),lr=(rl-rr)/std::max(rl+rr,std::numeric_limits<double>::epsilon());
  bool state_observed_finite=true;for(int id:{0,1,3,4,10,11,32})state_observed_finite &= std::isfinite(x[id]);
  bool good=state_observed_finite&&std::isfinite(o.ay)&&std::isfinite(fl)&&std::isfinite(fr)&&std::isfinite(rl)&&std::isfinite(rr)&&fl>0&&fr>0&&rl>0&&rr>0&&std::abs(lt)<p.ltr_limit&&std::abs(lf)<p.ltr_limit&&std::abs(lr)<p.ltr_limit&&std::abs(x[3])<p.roll_limit&&x[32]>0;
  bool h=p.handling(x[0],x[1]),y=std::abs(x[1])<=p.yaw_limit,a=std::abs(o.ay)<=p.ay_limit;
  if(!h&&inside)firstH=t;if(!good&&physical)firstPhysical=t;if(!a&&ay)firstAy=t;if(!y&&yaw)firstYaw=t;
  physical&=good;inside&=h;yaw&=y;ay&=a;
  minFz=std::min(minFz,std::min(std::min(fl,fr),std::min(rl,rr)));maxLtr=std::max(maxLtr,std::abs(lt));maxLtrF=std::max(maxLtrF,std::abs(lf));maxLtrR=std::max(maxLtrR,std::abs(lr));peakP=std::max(peakP,std::abs(x[10])/hmc::rad);peakPhi=std::max(peakPhi,std::abs(x[3])/hmc::rad);peakR=std::max(peakR,std::abs(x[1]));peakAy=std::max(peakAy,std::abs(o.ay));if(t==0)initialP=x[10]/hmc::rad;lastP=x[10]/hmc::rad;observed=t;
 }
};
std::array<double,NF> snapshot(const History&s,double err,bool trimOk,bool reached){bool recovered=reached&&s.physical&&err<=1,passed=recovered&&s.inside&&trimOk;return {double(passed),double(recovered),double(reached&&s.physical),double(passed&&s.yaw&&s.ay),s.firstH,s.firstPhysical,s.minFz,s.maxLtr,s.maxLtrF,s.maxLtrR,s.peakP,s.peakPhi,s.initialP,s.lastP,s.firstAy,s.firstYaw,s.peakR,s.peakAy,s.observed,err,double(s.inside),double(s.yaw),double(s.ay)};}
double terminal_error(const State&x,const State&t,const Policy&p){const int ids[7]={0,1,3,10,4,11,32};double e=0;for(int i=0;i<7;++i)e=std::max(e,std::abs(x[ids[i]]-t[ids[i]])/p.tol[i]);return e;}
}
int main(int argc,char**argv){try{
 if(argc<5)throw std::runtime_error("Usage:hmc_native rhs|integrate|rollout config.json states.bin result.bin [threads]");
 std::string mode=argv[1];std::ifstream cf(argv[2]);if(!cf)throw std::runtime_error("Cannot open config");std::ostringstream ss;ss<<cf.rdbuf();Json config=Json::parse(ss.str());hmc::Vehicle model(config.at("Q"));
 std::ifstream fi(argv[3],std::ios::binary|std::ios::ate);if(!fi)throw std::runtime_error("Cannot open states");auto bytes=fi.tellg();if(bytes<0||std::uint64_t(bytes)%(34*sizeof(double)))throw std::runtime_error("Bad statefile length");std::size_t n=std::uint64_t(bytes)/(34*sizeof(double));std::vector<State>states(n);fi.seekg(0);fi.read(reinterpret_cast<char*>(states.data()),bytes);if(!fi)throw std::runtime_error("State readfailed");
 int threads=argc>5?std::stoi(argv[5]):4;
#ifdef _OPENMP
 omp_set_num_threads(std::max(1,threads));
#endif
 std::ofstream out(argv[4],std::ios::binary);if(!out)throw std::runtime_error("Cannot open result");
 if(mode=="rhs"){
  std::vector<State>d(n);
#pragma omp parallel for schedule(static)
  for(std::int64_t i=0;i<std::int64_t(n);++i)model.rhs(states[i],d[i],nullptr);
  out.write(reinterpret_cast<const char*>(d.data()),d.size()*sizeof(State));
 }else if(mode=="trajectory"){
  if(n!=1)throw std::runtime_error("Trajectory mode requires one initial state");
  const auto&c=config.at("c");double dt=c.num("dt",.0005),duration=10;
  if(c.has("horizons"))duration=c.vec("horizons").back();if(c.has("duration"))duration=c.num("duration");
  std::int64_t steps=std::llround(duration/dt);if(dt<=0||std::abs(steps*dt-duration)>1e-9)throw std::runtime_error("Invalid trajectory duration");
  State x=states[0],dx;std::array<double,40>a;
  for(std::int64_t k=0;k<=steps;++k){hmc::Observation obs;model.rhs(x,dx,&obs);a[0]=k*dt;std::copy(x.begin(),x.end(),a.begin()+1);a[35]=obs.ay;std::copy(obs.fz.begin(),obs.fz.end(),a.begin()+36);out.write(reinterpret_cast<const char*>(a.data()),a.size()*sizeof(double));if(k<steps)x=rk4(x,dt,model);}
 }else if(mode=="integrate"){
  const auto&c=config.at("c");double dt=c.num("dt",.0005),duration=10;if(c.has("horizons"))duration=c.vec("horizons").back();if(c.has("horizons_s"))duration=c.vec("horizons_s").back();if(c.has("duration"))duration=c.num("duration");std::int64_t steps=std::llround(duration/dt);if(dt<=0||std::abs(steps*dt-duration)>1e-9)throw std::runtime_error("Invalid integrationduration");
#pragma omp parallel for schedule(dynamic)
  for(std::int64_t i=0;i<std::int64_t(n);++i){for(std::int64_t k=0;k<steps;++k){states[i]=rk4(states[i],dt,model);if(!finite(states[i]))break;}}
  out.write(reinterpret_cast<const char*>(states.data()),states.size()*sizeof(State));
 }else if(mode=="rollout"){
  Policy p(config);State target=trim_state(config.at("T"));bool trimOK=p.allow_trim_outside||p.handling(target[0],target[1]);std::size_t nh=p.horizons.size(),record=NF*nh+34;std::vector<double>results(n*record);
#pragma omp parallel for schedule(dynamic)
  for(std::int64_t i=0;i<std::int64_t(n);++i){State x=states[i],dx;History hist;std::vector<double>err(nh,0);std::size_t next=0;std::int64_t maxstep=std::llround(p.horizons.back()/p.dt);bool alive=true;
   for(std::int64_t k=0;k<=maxstep;++k){double t=k*p.dt;hmc::Observation obs;model.rhs(x,dx,&obs);hist.update(t,x,obs,p);double e=terminal_error(x,target,p);
    for(std::size_t j=next;j<nh;++j)if(t>=p.horizons[j]-p.window-1e-8&&t<=p.horizons[j]+1e-8)err[j]=std::max(err[j],e);
    if(next<nh&&k==std::llround(p.horizons[next]/p.dt)){auto a=snapshot(hist,err[next],trimOK,true);std::copy(a.begin(),a.end(),results.begin()+i*record+NF*next);++next;}
    if(k==maxstep)break;if(!finite(x)){alive=false;break;}x=rk4(x,p.dt,model);
   }
   if(!alive){hist.physical=false;for(std::size_t j=next;j<nh;++j){auto a=snapshot(hist,INFINITY,trimOK,false);std::copy(a.begin(),a.end(),results.begin()+i*record+NF*j);}}
   std::copy(x.begin(),x.end(),results.begin()+i*record+NF*nh);
  }
  out.write(reinterpret_cast<const char*>(results.data()),results.size()*sizeof(double));
 }else throw std::runtime_error("Unknown mode "+mode);
 if(!out)throw std::runtime_error("Result writefailed");return 0;
}catch(const std::exception&e){std::cerr<<"HMC native: "<<e.what()<<'\n';return 1;}}
