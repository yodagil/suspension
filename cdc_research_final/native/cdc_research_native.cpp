// New finite-horizon path validation around the supplied CDC physics.
// Compile without -ffast-math. No geometric shifting or trajectory projection.
#include "cdc_physics.hpp"
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
namespace research {
constexpr double INF=std::numeric_limits<double>::infinity();
constexpr int NF=26;
struct Config {
 Params p; double dt=.0005,window=1,yawfactor=.85,ayfactor=1,ltrlimit=1;
 bool usable=false,reject=true; int threads=4,trace_stride=0;
 std::array<double,6>guard{30,2,40*pi/180,.5,1.2,300};
 std::array<double,29>tol{};State trim{};std::vector<double> horizons;
 Config():p(50,.9,0,std::array<int,4>{0,0,0,0}.data()){}
};
Config readconfig(const std::string&path){
 std::ifstream f(path);std::string magic;f>>magic;
 if(magic!="CDC_RESEARCH_FIXED_V1")throw std::runtime_error("Wrong config version");
 Config c;double speed,mu,steer;int w[4],usable,reject,nh;
 f>>speed>>mu>>steer;for(int&i:w)f>>i;c.p=Params(speed,mu,steer,w);
 f>>c.dt>>c.window>>nh;c.horizons.resize(nh);for(auto&x:c.horizons)f>>x;
 f>>usable>>c.yawfactor>>c.ayfactor>>c.ltrlimit>>reject>>c.threads>>c.trace_stride;
 c.usable=usable;c.reject=reject;for(auto&x:c.guard)f>>x;
 for(auto&x:c.tol)f>>x;for(auto&x:c.trim)f>>x;
 for(auto &row:c.p.dm)for(double&x:row)f>>x;
 if(!f||c.dt<=0||nh<1||nh>100||c.window<=0||c.threads<1)throw std::runtime_error("Invalid config");
 double prev=0;for(double t:c.horizons){if(t<=prev||t<c.window||fabs(t/c.dt-round(t/c.dt))>1e-6)throw std::runtime_error("Horizon must increase on dt grid");prev=t;}
 for(double v:c.tol)if(v<=0||!std::isfinite(v))throw std::runtime_error("Invalid tolerance");
 return c;
}
std::vector<State> readstates(const std::string&p){std::ifstream f(p);size_t n;f>>n;if(!f||n<1||n>2000000)throw std::runtime_error("Invalid state count");std::vector<State>x(n);for(auto&a:x)for(auto&b:a)f>>b;if(!f)throw std::runtime_error("Invalid initial state file");return x;}
// Exact derivative of supplied pure lateral MF (alpha sign retained).
struct MF {double C,D,B,E,SH,SV,Cref;};
MF mf(double fz,int wear,double sy){
 double C,PD1,PD2,PE1,PE2,PK1,PK2,PH1,PH2,PV1,PV2;
 if(wear==0){C=1.5662;PD1=1.0278;PD2=-.17125;PE1=-1.1357;PE2=-1.0544;PK1=-25.85;PK2=2.1377;PH1=8.675e-4;PH2=7.046e-5;PV1=-.007239;PV2=-7.442e-5;}
 else if(wear==25){C=1.5457;PD1=1.0286;PD2=-.16688;PE1=-1.439;PE2=-1.6063;PK1=-26.;PK2=1.9164;PH1=1.82e-3;PH2=-7.193e-4;PV1=1.802e-3;PV2=-1.1497e-2;}
 else{C=1.5057;PD1=1.0067;PD2=-.2019;PE1=-2.;PE2=-2.;PK1=-27.27;PK2=1.5963;PH1=2.3735e-3;PH2=2.38e-4;PV1=.01487;PV2=-.011025;}
 double df=(fz-5425.)/5425.,D=(PD1+PD2*df)*fz*sy,K=PK1*5425.*sin(2*atan(fz/(PK2*5425.)));
 return {C,D,fabs(C*D)<1e-9?0:K/(C*D),std::min(1.,PE1+PE2*df),PH1+PH2*df,fz*(PV1+PV2*df),std::max(1.,fabs(K))};
}
inline void fy_slope(double a,const MF&q,double&fy,double&slope){double z=q.B*(a+q.SH),y=z-q.E*(z-atan(z));double ang=q.C*atan(y);fy=q.D*sin(ang)+q.SV;slope=q.D*cos(ang)*q.C/(1+y*y)*q.B*(1-q.E+q.E/(1+z*z));}
struct Handling{double margin,trace,det,minC,fv,fr;bool inside;};
Handling handling(double v,double r,const Params&p,const std::array<MF,4>&mm){
 double j11=0,j12=-p.ux,j21=0,j22=0,fyall=0,mz=0,minC=INF,minnorm=INF;bool domain=true;
 for(int i=0;i<4;i++){
  double ub=p.ux-yw[i]*r,vb=v+xw[i]*r,uw=p.cs[i]*ub+p.sn[i]*vb,vw=-p.sn[i]*ub+p.cs[i]*vb;
  domain=domain&&uw>.1;double a=atan2(vw,uw),fy,d;fy_slope(a,mm[i],fy,d);
  double den=ub*ub+vb*vb,av=ub/den,ar=(xw[i]*ub+yw[i]*vb)/den,b=p.cs[i]/m,z=(xw[i]*p.cs[i]+yw[i]*p.sn[i])/Iz;
  j11+=b*d*av;j12+=b*d*ar;j21+=z*d*av;j22+=z*d*ar;
  fyall+=fy*p.cs[i];mz+=(xw[i]*p.cs[i]+yw[i]*p.sn[i])*fy;minC=std::min(minC,-d);minnorm=std::min(minnorm,-d/mm[i].Cref);
 }
 double trc=j11+j22,det=j11*j22-j12*j21;
 double margin=std::min(std::min(-trc,det),minnorm);if(!domain||!std::isfinite(margin))margin=-INF;
 return {margin,trc,det,minC,fyall/m-p.ux*r,mz/Iz,domain&&trc<0&&det>0&&minC>0};
}
struct Obs{double ay,ltr,ltrf,ltrr,raw[4],elastic[4],minC;};
Obs observation(const State&x,const Params&p,const State&dx){
 Obs o{};o.ay=dx[0]+p.ux*x[1];o.minC=INF;double fz[4],el[4];loads(x,p,fz,el,o.raw);
 double L=o.raw[0]+o.raw[2],R=o.raw[1]+o.raw[3];o.ltr=(L-R)/(L+R);o.ltrf=(o.raw[0]-o.raw[1])/(o.raw[0]+o.raw[1]);o.ltrr=(o.raw[2]-o.raw[3])/(o.raw[2]+o.raw[3]);
 for(int i=0;i<4;i++){
  o.elastic[i]=p.fz0[i]-200000*x[5+i];double fx,fy,s,yp; tire(x[16+i],x[20+i],fz[i],p.wear[i],p,fx,fy);fy_slope(x[20+i],mf(fz[i],p.wear[i],p.sy),yp,s);
  double df=(fz[i]-5425.)/5425.,wx=p.wear[i]==0?1:(p.wear[i]==25?.95:.90);double cap=std::max(1.,std::max(.05,fabs(1.145*wx-.2106*df)*p.sx)*fz[i]);double rho=std::min(1.,fabs(fx)/cap);s*=sqrt(std::max(0.,1-rho*rho));if(fz[i]<=100)s=0;o.minC=std::min(o.minC,-s);
 }return o;
}
uint16_t codes(const State&x,const State&dx,const Obs&o,const Config&c){
 uint16_t b=0;if(fabs(x[1])>c.yawfactor*c.p.mu*g/c.p.ux)b|=1;if(fabs(o.ay)>c.ayfactor*c.p.mu*g)b|=2;
 if(fabs(o.ltr)>=c.ltrlimit)b|=4;if(fabs(o.ltrf)>=c.ltrlimit||fabs(o.ltrr)>=c.ltrlimit)b|=16;
 bool num=!std::isfinite(o.ay)||!std::isfinite(o.ltr)||!std::isfinite(o.ltrf)||!std::isfinite(o.ltrr);for(double v:x)num|=!std::isfinite(v);for(double v:dx)num|=!std::isfinite(v);
 bool guard=fabs(x[0])>c.guard[0]||fabs(x[1])>c.guard[1]||fabs(x[3])>c.guard[2],clip=false;
 for(int i=0;i<4;i++){if(o.raw[i]<=0)b|=8;num|=!std::isfinite(o.raw[i]);double uw=c.p.cs[i]*(c.p.ux-yw[i]*x[1])+c.p.sn[i]*(x[0]+xw[i]*x[1]);guard|=fabs(x[16+i])>c.guard[3]+1e-8||fabs(x[20+i])>c.guard[4]||x[24+i]<0||x[24+i]>c.guard[5]||uw<=0;clip|=o.raw[i]<=100||o.raw[i]>=16000||o.elastic[i]<=100||o.elastic[i]>=16000;}
 if(num)b|=32;if(guard)b|=64;if(clip&&c.reject)b|=128;return b;
}
using Row=std::array<double,NF>;
struct Result{std::vector<Row> rows;State last;std::vector<std::array<double,12>>trace;};
Result one(State x,const Config&c){
 size_t nh=c.horizons.size(),ih=0;Result out;out.rows.resize(nh);std::array<MF,4>mm;for(int i=0;i<4;i++)mm[i]=mf(c.p.fz0[i],c.p.wear[i],c.p.sy);
 int n=int(std::lround(c.horizons.back()/c.dt));uint16_t reason=0;
 double firsth=INF,firstay=INF,firstyaw=INF,firsthard=INF,firsts=INF,outside=0,minmargin=INF,minC=INF,peakr=0,peakay=0,peakltr=0,minfz=INF,lastnot=-INF,err=INF,derr=INF,observed=0;
 bool alive=true,kept=true,prepeak=true,previous_inside=true;
 for(int k=0;k<=n;k++){
  double t=k*c.dt;if(k&&!previous_inside)outside+=c.dt;
  State f;rhs(x,c.p,f);Obs o=observation(x,c.p,f);uint16_t bits=codes(x,f,o,c);reason|=bits;
  Handling h=handling(x[0],x[1],c.p,mm);if(!h.inside){kept=false;firsth=std::min(firsth,t);}previous_inside=h.inside;minmargin=std::min(minmargin,h.margin);
  minC=std::min(minC,o.minC);if(!(o.minC>0)){prepeak=false;firsts=std::min(firsts,t);}
  if(bits&2)firstay=std::min(firstay,t);if(bits&1)firstyaw=std::min(firstyaw,t);
  if(bits&252){alive=false;firsthard=std::min(firsthard,t);}
  peakr=std::max(peakr,fabs(x[1]));peakay=std::max(peakay,fabs(o.ay));peakltr=std::max(peakltr,std::max(fabs(o.ltr),std::max(fabs(o.ltrf),fabs(o.ltrr))));for(double v:o.raw)minfz=std::min(minfz,v);
  observed=t;if(c.usable){err=0;derr=0;for(int j=0;j<29;j++){err=std::max(err,fabs(x[j]-c.trim[j])/c.tol[j]);derr=std::max(derr,fabs(f[j])/c.tol[j]);}if(err>1||derr>1)lastnot=t;}
  if(c.trace_stride>0&&(k%c.trace_stride==0||k==n||!alive))out.trace.push_back({t,x[0],x[1],x[3],x[10],o.ay,o.ltr,minfz,h.margin,o.minC,err,derr});
  auto snapshot=[&](bool completed){bool ms=alive&&completed,sa=ms&&!(reason&2),ya=!(reason&1),close=c.usable&&t-lastnot>=c.window-1e-10&&err<=1&&derr<=1;bool mr=ms&&close,re=sa&&close,hp=re&&kept;
   return Row{double(sa),double(ms),double(ya),double(mr),double(re),double(re&&ya),double(hp),double(hp&&ya),double(hp&&prepeak),err,derr,firsth,outside,minmargin,firstay,firstyaw,firsthard,peakr,peakay,peakltr,minfz,minC,firsts,double(reason),observed,double(kept)};};
  if(ih<nh&&k==int(std::lround(c.horizons[ih]/c.dt))){out.rows[ih++]=snapshot(true);}
  if(!alive){while(ih<nh)out.rows[ih++]=snapshot(false);break;}
  if(k==n)break;State f2,f3,f4,z;for(int j=0;j<32;j++)z[j]=x[j]+.5*c.dt*f[j];rhs(z,c.p,f2);for(int j=0;j<32;j++)z[j]=x[j]+.5*c.dt*f2[j];rhs(z,c.p,f3);for(int j=0;j<32;j++)z[j]=x[j]+c.dt*f3[j];rhs(z,c.p,f4);for(int j=0;j<32;j++)x[j]+=c.dt*(f[j]+2*f2[j]+2*f3[j]+f4[j])/6;
 }
 out.last=x;return out;
}
void write(const std::string&path,const std::vector<Result>&out,int nh){std::ofstream f(path,std::ios::binary);uint32_t header[4]={0x43524331,uint32_t(out.size()),uint32_t(nh),NF};f.write((char*)header,sizeof(header));for(const auto&r:out){for(const auto&row:r.rows)f.write((char*)row.data(),NF*sizeof(double));f.write((char*)r.last.data(),32*sizeof(double));}if(!f)throw std::runtime_error("Output write failed");}
}
int main(int argc,char**argv){using namespace research;try{if(argc<5)throw std::runtime_error("Usage: executable simulate|rhs|handling config states output");auto c=readconfig(argv[2]);auto states=readstates(argv[3]);std::string mode=argv[1];
 if(mode=="rhs"||mode=="handling"){std::ofstream f(argv[4]);f<<std::setprecision(17);std::array<MF,4>mm;for(int i=0;i<4;i++)mm[i]=mf(c.p.fz0[i],c.p.wear[i],c.p.sy);for(const auto&x:states){if(mode=="rhs"){State d;rhs(x,c.p,d);for(double v:d)f<<v<<' ';}else{auto h=handling(x[0],x[1],c.p,mm);f<<h.inside<<' '<<h.margin<<' '<<h.trace<<' '<<h.det<<' '<<h.minC<<' '<<h.fv<<' '<<h.fr;}f<<'\n';}return 0;}
 if(mode!="simulate")throw std::runtime_error("Unknown operation");std::vector<Result>r(states.size());omp_set_num_threads(c.threads);
 #pragma omp parallel for schedule(dynamic,4)
 for(size_t i=0;i<states.size();i++)r[i]=one(states[i],c);
 write(argv[4],r,int(c.horizons.size()));if(c.trace_stride>0){std::ofstream f(std::string(argv[4])+".trace.csv");f<<"point_index,t,vy,r,phi,p,ay,LTR,min_Fz_so_far,handling_margin,min_restoring_slope,error,derivative\n"<<std::setprecision(17);for(size_t i=0;i<r.size();i++)for(const auto&v:r[i].trace){f<<i+1;for(double z:v)f<<','<<z;f<<'\n';}}
 return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
