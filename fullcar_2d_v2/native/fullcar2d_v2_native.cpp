// V2 full-car conditional 2-D initial-state sections with uploaded V2 criteria.
// The included 32-state equations are unchanged; there is NO state projection.
#define main frozen_driver_main
#include "frozen_native_model.cpp"
#undef main
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>

namespace cdc2d {
constexpr double inf=std::numeric_limits<double>::infinity();
struct Config {
 Params p;double speed=50,mu=.9,steer=0,window=1,derivscale=1;
 int batch=256,init=2,slice=0;bool root=false,usable=false,reject=true;
 double yawfactor=.85,ayfactor=1,ltrlimit=1;
 std::array<double,6>guard{30,2,40*pi/180,.5,1.2,300};
 std::array<double,29>tol{};State trim{};std::vector<double>horizons;
 std::vector<std::array<double,4>>variants;
 Config():p(50,.9,0,std::array<int,4>{0,0,0,0}.data()){}
};
Config readconfig(const std::string&path){
 std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open config");
 std::string magic;f>>magic;if(magic!="FULLCAR2D_V2")throw std::runtime_error("Wrong config schema");
 Config c;int w[4],root,usable,reject,nh,nv;
 f>>c.speed>>c.mu>>c.steer;for(int&i:w)f>>i;
 c.p=Params(c.speed,c.mu,c.steer,w);
 f>>c.p.Ts>>c.p.sub>>c.window>>c.derivscale>>c.batch>>c.init>>root>>usable>>c.slice;
 c.root=root;c.usable=usable;
 f>>c.yawfactor>>c.ayfactor>>c.ltrlimit>>reject;c.reject=reject;
 for(double&x:c.guard)f>>x;
 f>>nh;if(nh<1||nh>100)throw std::runtime_error("Invalid horizon count");
 c.horizons.resize(nh);for(double&x:c.horizons)f>>x;
 for(double&x:c.tol)f>>x;for(double&x:c.trim)f>>x;
 f>>nv;if(nv<1||nv>100)throw std::runtime_error("Invalid variant count");
 c.variants.resize(nv);for(auto&v:c.variants)for(double&x:v)f>>x;
 if(!f||c.p.Ts<=0||c.p.sub<1||c.batch<1||c.init<0||c.init>2||c.slice<0||c.slice>2||c.window<=0||c.derivscale<=0)
  throw std::runtime_error("Invalid/incomplete config");
 double previous=0;for(double h:c.horizons){double k=h/(c.p.Ts/c.p.sub);
  if(h<previous||h<c.window||fabs(k-std::round(k))>1e-7)throw std::runtime_error("Invalid horizon grid");previous=h;}
 for(double t:c.tol)if(t<=0||!std::isfinite(t))throw std::runtime_error("Invalid terminal tolerance");
 return c;
}
using Point=std::array<double,3>;
std::vector<Point>readpoints(const std::string&path){
 std::ifstream f(path);size_t n;f>>n;if(!f||n<1||n>10000000)throw std::runtime_error("Invalid points file");
 std::vector<Point>pts(n);for(auto&q:pts)for(double&v:q)f>>v;
 if(!f)throw std::runtime_error("Incomplete points file");return pts;
}
State lift(const Point&q,const Config&c,const std::array<double,4>&v){
 State x{};if(c.init==2&&c.root)x=c.trim;
 x[0]=q[0];x[1]=q[1];
 if(c.slice==1){for(int j=2;j<16;j++)x[j]=0;x[28]=0;x[3]=q[2];}
 else if(c.slice==2||c.init!=2||!c.root)x[3]=q[2];
 x[29]=x[30]=x[31]=0;
 if(c.init==0){for(int i=0;i<4;i++){x[16+i]=0;x[20+i]=-c.p.delta[i];x[24+i]=std::max(c.p.ux*c.p.cs[i],0.)/Rw;}}
 else{
  double fz[4],el[4],raw[4];loads(x,c.p,fz,el,raw);
  for(int i=0;i<4;i++){
   double ub=c.p.ux-x[1]*yw[i],vb=x[0]+x[1]*xw[i];
   double uw=c.p.cs[i]*ub+c.p.sn[i]*vb,vw=-c.p.sn[i]*ub+c.p.cs[i]*vb;
   double us=uw;if(us>=0&&us<.1)us=.1;if(us<0&&us>-.1)us=-.1;
   double kap=.001635-.0025297*((fz[i]-5425.)/5425.);
   x[16+i]=kap;x[20+i]=atan2(vw,us);x[24+i]=(uw+kap*std::max(fabs(uw),.5))/Rw;
  }
 }
 x[10]+=v[0];x[20]+=v[1];x[21]+=v[1];x[22]+=v[2];x[23]+=v[2];x[28]+=v[3];return x;
}
struct Obs{double ay=0,ltr=0,ltrf=0,ltrr=0,raw[4]{},elastic[4]{},uw[4]{};};
Obs observation(const State&x,const Config&c){
 Obs o;double fz[4],el[4],fxb[4],fyb[4],fxt[4];loads(x,c.p,fz,el,o.raw);forces(x,c.p,fz,fxb,fyb,fxt);
 o.ay=(fyb[0]+fyb[1]+fyb[2]+fyb[3])/m;
 double L=o.raw[0]+o.raw[2],R=o.raw[1]+o.raw[3],den=L+R;
 o.ltr=(L-R)/den;if(fabs(den)<1e-9)o.ltr=fabs(L-R)<1e-9?0:inf;
 o.ltrf=(o.raw[0]-o.raw[1])/(o.raw[0]+o.raw[1]);
 o.ltrr=(o.raw[2]-o.raw[3])/(o.raw[2]+o.raw[3]);
 for(int i=0;i<4;i++){
  o.elastic[i]=c.p.fz0[i]-200000*x[5+i];
  o.uw[i]=c.p.cs[i]*(c.p.ux-x[1]*yw[i])+c.p.sn[i]*(x[0]+x[1]*xw[i]);
 }return o;
}
uint16_t codes(const State&x,const Obs&o,const Config&c,const State&dx){
 uint16_t b=0;
 if(fabs(x[1])>c.yawfactor*c.mu*g/c.p.ux)b|=1;
 if(fabs(o.ay)>c.ayfactor*c.mu*g)b|=2;
 if(fabs(o.ltr)>=c.ltrlimit)b|=4;
 if(fabs(o.ltrf)>=c.ltrlimit||fabs(o.ltrr)>=c.ltrlimit)b|=16;
 bool num=!std::isfinite(o.ay)||!std::isfinite(o.ltr)||!std::isfinite(o.ltrf)||!std::isfinite(o.ltrr);
 for(double v:x)num=num||!std::isfinite(v);for(double v:dx)num=num||!std::isfinite(v);
 bool guard=fabs(x[0])>c.guard[0]||fabs(x[1])>c.guard[1]||fabs(x[3])>c.guard[2],clip=false;
 for(int i=0;i<4;i++){
  if(o.raw[i]<=0)b|=8;num=num||!std::isfinite(o.raw[i]);
  guard=guard||fabs(x[16+i])>c.guard[3]+1e-8||fabs(x[20+i])>c.guard[4]||x[24+i]<0||x[24+i]>c.guard[5]||o.uw[i]<=0;
  clip=clip||o.raw[i]<=100||o.raw[i]>=16000||o.elastic[i]<=100||o.elastic[i]>=16000;
 }
 if(num)b|=32;if(guard)b|=64;if(clip&&c.reject)b|=128;return b;
}
void rk4(State&x,const Params&p,double dt,const State&k1){
 State k2,k3,k4,z;
 for(int i=0;i<32;i++)z[i]=x[i]+.5*dt*k1[i];rhs(z,p,k2);
 for(int i=0;i<32;i++)z[i]=x[i]+.5*dt*k2[i];rhs(z,p,k3);
 for(int i=0;i<32;i++)z[i]=x[i]+dt*k3[i];rhs(z,p,k4);
 for(int i=0;i<32;i++)x[i]+=dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
}
struct Record{
 std::vector<double>safe,yaw,recovered,reason,error,deriv;
 double firstbad=inf,firstyaw=inf,peakr=0,peakay=0,peakltr=0,minfz=inf;
 int failed=0;long long observations=0;State last{};
 Record(size_t nh):safe(nh,0),yaw(nh,0),recovered(nh,0),reason(nh,0),error(nh,inf),deriv(nh,inf){}
};
Record one(const Point&q,const Config&c,const std::array<double,4>&variant){
 size_t nh=c.horizons.size(),ih=0;Record out(nh);State x=lift(q,c,variant);
 double dt=c.p.Ts/c.p.sub;int last=int(std::lround(c.horizons.back()/dt));
 std::vector<int>H(nh);for(size_t j=0;j<nh;j++)H[j]=int(std::lround(c.horizons[j]/dt));
 uint16_t reason=0;double err=inf,derr=inf,lastnot=-inf;out.failed=last+1;
 for(int k=0;k<=last;k++){
  double t=k*dt;State dx;rhs(x,c.p,dx);Obs o=observation(x,c);uint16_t b=codes(x,o,c,dx);reason|=b;out.observations++;
  if(b&1)out.firstyaw=std::min(out.firstyaw,t);
  out.peakr=std::max(out.peakr,fabs(x[1]));out.peakay=std::max(out.peakay,fabs(o.ay));
  out.peakltr=std::max(out.peakltr,std::max(fabs(o.ltr),std::max(fabs(o.ltrf),fabs(o.ltrr))));
  for(double v:o.raw)out.minfz=std::min(out.minfz,v);
  if(c.usable){err=0;derr=0;for(int i=0;i<29;i++){
   err=std::max(err,fabs(x[i]-c.trim[i])/c.tol[i]);derr=std::max(derr,fabs(dx[i])/(c.tol[i]/c.derivscale));}
   if(err>1||derr>1)lastnot=t;
  }
  bool bad=(b&254)!=0;
  if(ih<nh&&k==H[ih]){
   out.safe[ih]=!bad;out.yaw[ih]=(reason&1)==0;
   out.recovered[ih]=!bad&&c.usable&&(t-lastnot>=c.window-1e-10)&&err<=1&&derr<=1;
   out.reason[ih]=reason;out.error[ih]=err;out.deriv[ih]=derr;ih++;
  }
  if(bad){out.failed=k;out.firstbad=t;for(size_t j=ih;j<nh;j++){
    out.yaw[j]=(reason&1)==0;out.reason[j]=reason;out.error[j]=err;out.deriv[j]=derr;
   }break;}
  if(k<last)rk4(x,c.p,dt,dx);
 }
 out.last=x;return out;
}
std::vector<Record>simulate(const std::vector<Point>&points,const Config&c,int threads){
 const size_t n=points.size(),nh=c.horizons.size();std::vector<Record>out;out.reserve(n);
 for(size_t i=0;i<n;i++){out.emplace_back(nh);std::fill(out.back().safe.begin(),out.back().safe.end(),1);std::fill(out.back().yaw.begin(),out.back().yaw.end(),1);std::fill(out.back().recovered.begin(),out.back().recovered.end(),1);std::fill(out.back().error.begin(),out.back().error.end(),0);std::fill(out.back().deriv.begin(),out.back().deriv.end(),0);}
 omp_set_num_threads(threads);
 for(const auto&variant:c.variants){
  std::vector<Record>current;current.reserve(n);for(size_t i=0;i<n;i++)current.emplace_back(nh);
  #pragma omp parallel for schedule(dynamic,4)
  for(size_t i=0;i<n;i++)current[i]=one(points[i],c,variant);
  // Reproduce uploaded simulate_batch's diagnostic-only early-break behavior.
  for(size_t first=0;first<n;first+=c.batch){size_t stop=std::min(n,first+size_t(c.batch));int lastfailure=0;
   for(size_t i=first;i<stop;i++)lastfailure=std::max(lastfailure,current[i].failed);
   for(size_t h=0;h<nh;h++)if(lastfailure<int(std::lround(c.horizons[h]/(c.p.Ts/c.p.sub))))
    for(size_t i=first;i<stop;i++)current[i].error[h]=current[i].deriv[h]=inf;
  }
  for(size_t i=0;i<n;i++){auto&a=out[i];const auto&b=current[i];
   for(size_t h=0;h<nh;h++){a.safe[h]=bool(a.safe[h])&&bool(b.safe[h]);a.yaw[h]=bool(a.yaw[h])&&bool(b.yaw[h]);a.recovered[h]=bool(a.recovered[h])&&bool(b.recovered[h]);a.reason[h]=uint16_t(a.reason[h])|uint16_t(b.reason[h]);a.error[h]=std::max(a.error[h],b.error[h]);a.deriv[h]=std::max(a.deriv[h],b.deriv[h]);}
   a.firstbad=std::min(a.firstbad,b.firstbad);a.firstyaw=std::min(a.firstyaw,b.firstyaw);
   a.peakr=std::max(a.peakr,b.peakr);a.peakay=std::max(a.peakay,b.peakay);a.peakltr=std::max(a.peakltr,b.peakltr);a.minfz=std::min(a.minfz,b.minfz);a.observations+=b.observations;a.last=b.last;
  }
 }return out;
}
void binary(const std::string&path,const std::vector<Record>&rows,size_t nh){
 uint16_t endian=1;if(*reinterpret_cast<char*>(&endian)!=1)throw std::runtime_error("Little-endian platform required");
 std::ifstream exists(path);if(exists.good())throw std::runtime_error("Refusing existing output");
 std::ofstream f(path,std::ios::binary);uint32_t h[4]={0x46435632,2,uint32_t(rows.size()),uint32_t(nh)};f.write(reinterpret_cast<char*>(h),sizeof(h));
 for(int field=0;field<6;field++)for(size_t j=0;j<nh;j++)for(const auto&r:rows){
  const auto&v=field==0?r.safe:field==1?r.yaw:field==2?r.recovered:field==3?r.reason:field==4?r.error:r.deriv;double x=v[j];f.write(reinterpret_cast<char*>(&x),sizeof(x));}
 for(int field=0;field<6;field++)for(const auto&r:rows){double x=field==0?r.firstbad:field==1?r.firstyaw:field==2?r.peakr:field==3?r.peakay:field==4?r.peakltr:r.minfz;f.write(reinterpret_cast<char*>(&x),sizeof(x));}
 if(!f)throw std::runtime_error("Native output write failed");
}
void num(std::ostream&o,double x){if(std::isfinite(x))o<<x;else if(std::isinf(x))o<<(x>0?"\"Inf\"":"\"-Inf\"");else o<<"null";}
template<class C>void array(std::ostream&o,const C&v){o<<'[';bool first=true;for(double x:v){if(!first)o<<',';first=false;num(o,x);}o<<']';}
void json(const std::vector<Record>&rows){std::cout<<std::setprecision(17)<<"[";bool first=true;for(const auto&r:rows){if(!first)std::cout<<',';first=false;std::cout<<'{';
 const char*names[]={"safe","yaw_ok","recovered","reason","terminal_error","terminal_derivative"};
 const std::vector<double>*v[]={&r.safe,&r.yaw,&r.recovered,&r.reason,&r.error,&r.deriv};
 for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<'"'<<names[i]<<"\":";array(std::cout,*v[i]);}
 std::cout<<",\"first_physical_violation\":";num(std::cout,r.firstbad);std::cout<<",\"first_yaw_violation\":";num(std::cout,r.firstyaw);
 std::cout<<",\"peak_r\":";num(std::cout,r.peakr);std::cout<<",\"peak_ay\":";num(std::cout,r.peakay);std::cout<<",\"peak_LTR\":";num(std::cout,r.peakltr);std::cout<<",\"min_Fz\":";num(std::cout,r.minfz);std::cout<<",\"state_final32\":";array(std::cout,r.last);std::cout<<'}';}std::cout<<"]\n";}
}
int main(int argc,char**argv){using namespace cdc2d;try{
 std::string config,points,output,probe;int threads=4;
 for(int i=1;i<argc;i++){std::string a=argv[i];if(i+1>=argc)throw std::runtime_error("Missing argument");
  std::string value=argv[++i];if(a=="--config")config=value;else if(a=="--points")points=value;else if(a=="--output")output=value;else if(a=="--probe")probe=value;else if(a=="--threads")threads=std::stoi(value);else throw std::runtime_error("Unknown option "+a);}
 Config c=readconfig(config);if(threads<1)throw std::runtime_error("Invalid thread count");std::cout<<std::setprecision(17);
 if(probe=="rhs"||probe=="step"){
  State x{},dx;for(double&v:x)if(!(std::cin>>v))throw std::runtime_error("Need32stdinstatevalues");rhs(x,c.p,dx);
  if(probe=="step"){rk4(x,c.p,c.p.Ts/c.p.sub,dx);array(std::cout,x);}else array(std::cout,dx);std::cout<<'\n';return 0;}
 auto pts=readpoints(points);
 if(probe=="initial"){std::cout<<'[';for(size_t i=0;i<pts.size();i++){if(i)std::cout<<',';array(std::cout,lift(pts[i],c,c.variants.front()));}std::cout<<"]\n";return 0;}
 auto start=std::chrono::steady_clock::now();auto rows=simulate(pts,c,threads);
 if(probe=="trajectory"){json(rows);return 0;}
 if(!probe.empty())throw std::runtime_error("Unknown probe");if(output.empty())throw std::runtime_error("Need output path");binary(output,rows,c.horizons.size());
 long long observations=0;for(const auto&r:rows)observations+=r.observations;
 std::cout<<"{\"points\":"<<rows.size()<<",\"observations\":"<<observations<<",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"}\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
