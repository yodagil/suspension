// Actual steady-state simulations using the unchanged completed-sweep model.
// Only suspension spring commands are selected here; CDC current remains 950 mA.
#define main frozen_grid_driver_main
#include "model/native_accel.cpp"
#undef main
#include <deque>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace steady_lookup {
constexpr int dynamic_size=29, window_steps=200, check_steps=200;
const std::array<double,29> tolerance={
 1e-4,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,
 1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,
 1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,
 1e-3,1e-3,1e-3,1e-3,1e-4};
constexpr int wear_cases[5][4]={{0,0,0,0},{25,25,0,0},{50,50,25,25},
                              {0,0,25,25},{25,25,50,50}};
struct Input {long long row_id=0;double speed=50,mu=.9,steer=0;int wear_id=1,mode_id=1;std::string mode="nnnn";};
struct Sample {State x,dx;int reason;double t;bool clipping;};
struct Metrics {
 std::array<double,29> mean{},ptp{},rhsmax{},mean_diff{};
 double ptp_scaled=std::numeric_limits<double>::infinity();
 double rhs_scaled=std::numeric_limits<double>::infinity();
 double drift_scaled=std::numeric_limits<double>::infinity();
 int reason=0;bool clipping=false,complete=false;double start=0,end=0;
};
struct Result {
 Input in;State last{};Metrics tail;std::string status="not_converged_time_limit";
 bool converged=false,first5_complete=false;double end=0;int reason_total=0,first5_reason=0;
 int observations=0,consecutive_good=0;bool clipping_total=false;
 std::array<double,4> springs{};double seconds=0;
};
std::string mode_string(int index){std::string s(4,'n');const char* labels="nsh";
 for(int i=3;i>=0;i--){s[i]=labels[index%3];index/=3;}return s;}
int mode_index(const std::string&s){if(s.size()!=4)throw std::runtime_error("mode must have four n/s/h labels");
 int n=0;for(char c:s){size_t k=std::string("nsh").find(c);if(k==std::string::npos)throw std::runtime_error("invalid mode label");n=3*n+int(k);}return n;}
std::array<double,4> mode_springs(const std::string&s){mode_index(s);std::array<double,4>k{};
 for(int i=0;i<4;i++){k[i]=s[i]=='n'?30790:(s[i]=='s'?(i<2?22161:23632):(i<2?36086:35007));}return k;}
Input row_input(long long id){Input in;in.row_id=id;long long q=id;
 const double road_mu[5]={.9,.8,.7,.6,.5};
 in.mode_id=int(q%81)+1;in.mode=mode_string(int(q%81));q/=81;
 in.wear_id=int(q%5)+1;q/=5;in.steer=.5*int(q%7);q/=7;
 in.mu=road_mu[q%5];q/=5;in.speed=50+10*q;return in;}
long long input_row_id(const Input&in){int speed=int(std::lround((in.speed-50)/10)),mu=int(std::lround((.9-in.mu)/.1)),steer=int(std::lround(in.steer/.5));
 if(speed<0||speed>5||mu<0||mu>4||steer<0||steer>6||fabs(in.speed-(50+10*speed))>1e-9||fabs(in.mu-(.9-.1*mu))>1e-9||fabs(in.steer-.5*steer)>1e-9||in.wear_id<1||in.wear_id>5)throw std::runtime_error("case outside the requested 210 conditions / five wear cases");
 return ((((speed*5LL+mu)*7+steer)*5+in.wear_id-1)*81+mode_index(in.mode));}
bool all_finite(const State&x){for(double v:x)if(!std::isfinite(v))return false;return true;}
bool clipping_active(const State&x,const State&dx){
 for(int j=16;j<20;j++)if((x[j]<=-.5+1e-12&&dx[j]<0)||(x[j]>=.5-1e-12&&dx[j]>0))return true;
 for(int j=20;j<24;j++)if((x[j]<=-1.2+1e-12&&dx[j]<0)||(x[j]>=1.2-1e-12&&dx[j]>0))return true;
 for(int j=24;j<28;j++)if((x[j]<=1e-12&&dx[j]<0)||(x[j]>=300-1e-12&&dx[j]>0))return true;
 return false;
}
Metrics metrics(const std::deque<Sample>&samples){Metrics a;if(samples.empty())return a;
 a.start=samples.front().t;a.end=samples.back().t;a.complete=samples.size()==window_steps+1;
 a.ptp_scaled=0;a.rhs_scaled=0;a.drift_scaled=0;
 for(int j=0;j<dynamic_size;j++){
  double low=std::numeric_limits<double>::infinity(),high=-low,sum=0,left=0,right=0;size_t nl=0,nr=0;
  for(size_t n=0;n<samples.size();n++){const auto&s=samples[n];double v=s.x[j];low=std::min(low,v);high=std::max(high,v);sum+=v;
   a.rhsmax[j]=std::max(a.rhsmax[j],fabs(s.dx[j]));
   if(n<samples.size()/2){left+=v;nl++;}else{right+=v;nr++;}}
  a.mean[j]=sum/samples.size();a.ptp[j]=high-low;a.mean_diff[j]=nl&&nr?fabs(left/nl-right/nr):std::numeric_limits<double>::infinity();
  a.ptp_scaled=std::max(a.ptp_scaled,a.ptp[j]/tolerance[j]);
  a.rhs_scaled=std::max(a.rhs_scaled,a.rhsmax[j]/tolerance[j]);
  a.drift_scaled=std::max(a.drift_scaled,a.mean_diff[j]/tolerance[j]);
 }
 for(const auto&s:samples){a.reason|=s.reason;a.clipping=a.clipping||s.clipping;}
 return a;
}
Result run(const Input&in,double max_time){auto began=std::chrono::steady_clock::now();Result out;out.in=in;out.springs=mode_springs(in.mode);
 Params p(in.speed,in.mu,in.steer,wear_cases[in.wear_id-1]);p.time=max_time;p.Ts=.01;p.sub=20;p.early=0;
 for(int i=0;i<4;i++)p.kw[i]=out.springs[i]*MR[i]*MR[i];
 State x=initial(p,0,0,0);std::deque<Sample>samples;int max_steps=int(std::lround(max_time/p.Ts));
 for(int k=0;k<=max_steps;k++){
  out.end=k*p.Ts;out.last=x;out.observations++;
  if(!all_finite(x)){out.reason_total|=8;out.status="numerical_failure";break;}
  int reason=observe(x,p);State dx{};rhs(x,p,dx);if(!all_finite(dx))reason|=8;
  out.reason_total|=reason;if(k<=500)out.first5_reason|=reason;if(k>=500)out.first5_complete=true;
  if(reason&8){out.status="numerical_failure";break;}
  bool clipping=clipping_active(x,dx);out.clipping_total=out.clipping_total||clipping;
  samples.push_back({x,dx,reason,out.end,clipping});if(samples.size()>window_steps+1)samples.pop_front();
  if(reason&16){out.status="hard_guard";break;}
  if(k>=500&&(k-500)%check_steps==0){
   Metrics a=metrics(samples);bool good=a.complete&&a.ptp_scaled<=1&&a.rhs_scaled<=1&&a.drift_scaled<=.5&&!a.clipping;
   out.consecutive_good=good?out.consecutive_good+1:0;
   if(out.consecutive_good>=2){out.converged=true;out.status="converged";break;}
  }
  if(k<max_steps&&!step(x,p)){out.last=x;out.end=(k+1)*p.Ts;out.reason_total|=8;out.status="numerical_failure";break;}
 }
 out.tail=metrics(samples);out.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();return out;
}
void number(std::ostream&o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
template<class C>void array(std::ostream&o,const C&v){o<<'[';bool first=true;for(double x:v){if(!first)o<<',';first=false;number(o,x);}o<<']';}
std::string json(const Result&r){std::ostringstream o;o<<std::setprecision(17);const auto&a=r.tail;
 o<<"{\"row_id\":"<<r.in.row_id<<",\"speed_kmh\":"<<r.in.speed<<",\"mu_road\":"<<r.in.mu<<",\"steer_deg\":"<<r.in.steer<<",\"wear_id\":"<<r.in.wear_id<<",\"mode_id\":"<<r.in.mode_id<<",\"mode\":\""<<r.in.mode<<"\",\"wear\":";array(o,wear_cases[r.in.wear_id-1]);
 o<<",\"k_spring_N_m\":";array(o,r.springs);o<<",\"I_mA\":[950,950,950,950],\"status\":\""<<r.status<<"\",\"converged\":"<<(r.converged?"true":"false")<<",\"end_time_s\":"<<r.end<<",\"observations\":"<<r.observations<<",\"reasons_total\":"<<r.reason_total<<",\"first5s_reason\":"<<r.first5_reason<<",\"first5s_complete\":"<<(r.first5_complete?"true":"false")<<",\"lastwindow_reason\":"<<a.reason;
 o<<",\"consecutive_good_windows\":"<<r.consecutive_good<<",\"window_complete\":"<<(a.complete?"true":"false")<<",\"window_start_s\":"<<a.start<<",\"window_end_s\":"<<a.end<<",\"clipping_total\":"<<(r.clipping_total?"true":"false")<<",\"clipping_lastwindow\":"<<(a.clipping?"true":"false");
 o<<",\"state_final32\":";array(o,r.last);o<<",\"state_mean29\":";array(o,a.mean);o<<",\"p2p29\":";array(o,a.ptp);o<<",\"rhs_max_abs29\":";array(o,a.rhsmax);o<<",\"adjacent_mean_difference29\":";array(o,a.mean_diff);
 o<<",\"state_tolerance29\":";array(o,tolerance);o<<",\"rhs_tolerance29\":";array(o,tolerance);o<<",\"max_scaled_p2p\":";number(o,a.ptp_scaled);o<<",\"max_scaled_rhs\":";number(o,a.rhs_scaled);o<<",\"max_scaled_mean_drift\":";number(o,a.drift_scaled);o<<",\"wall_seconds\":"<<r.seconds<<'}';return o.str();
}
}
int main(int argc,char**argv){using namespace steady_lookup;
 try{long long start=0,stop=85050;int threads=4;double max_time=60;bool single=false;Input in;std::string output,operation;
  for(int i=1;i<argc;i++){std::string arg=argv[i];auto need=[&](int n){if(i+n>=argc)throw std::runtime_error("missing arguments for "+arg);};
   if(arg=="--start"){need(1);start=std::stoll(argv[++i]);}
   else if(arg=="--stop"){need(1);stop=std::stoll(argv[++i]);}
   else if(arg=="--threads"){need(1);threads=std::stoi(argv[++i]);}
   else if(arg=="--max-time"){need(1);max_time=std::stod(argv[++i]);}
   else if(arg=="--output"){need(1);output=argv[++i];}
   else if(arg=="--case"||arg=="--rhs"||arg=="--step"){need(5);single=true;operation=arg;in.speed=std::stod(argv[++i]);in.mu=std::stod(argv[++i]);in.steer=std::stod(argv[++i]);in.wear_id=std::stoi(argv[++i]);in.mode=argv[++i];in.mode_id=mode_index(in.mode)+1;in.row_id=input_row_id(in);}
   else throw std::runtime_error("unknown option "+arg);
  }
  if(!std::isfinite(max_time)||max_time<7||fabs(max_time*100-std::round(max_time*100))>1e-8||threads<1||start<0||stop>85050||stop<=start)throw std::runtime_error("invalid time, thread count, or half-open row range");
  if(single){
   if(operation=="--rhs"||operation=="--step"){
    Params p(in.speed,in.mu,in.steer,wear_cases[in.wear_id-1]);auto springs=mode_springs(in.mode);
    for(int i=0;i<4;i++)p.kw[i]=springs[i]*MR[i]*MR[i];
    p.Ts=.01;p.sub=20;State x{},y{};for(double&v:x)if(!(std::cin>>v))throw std::runtime_error("need 32-state stdin");
    if(operation=="--rhs")rhs(x,p,y);else{if(!step(x,p))throw std::runtime_error("nonfinite step");y=x;}
    std::cout<<std::setprecision(17);for(double v:y)std::cout<<v<<' ';std::cout<<'\n';return 0;
   }
   std::cout<<json(run(in,max_time))<<'\n';return 0;
  }
  if(output.empty())throw std::runtime_error("batch requires --output path; case example: --case 80 .9 2 1 nnnn");
  {std::ifstream exists(output);if(exists.good())throw std::runtime_error("refusing to overwrite existing output file");}
  std::ofstream stream(output);if(!stream)throw std::runtime_error("cannot open output file");omp_set_num_threads(threads);long long completed=0;
  #pragma omp parallel for schedule(dynamic,1)
  for(long long id=start;id<stop;id++){Result r=run(row_input(id),max_time);std::string line=json(r);
   #pragma omp critical(steady_lookup_output)
   {stream<<line<<'\n';stream.flush();completed++;if(completed==1||completed%100==0||completed==stop-start)std::cerr<<"COMPLETE "<<completed<<'/'<<stop-start<<" row_id="<<id<<" status="<<r.status<<'\n';}
  }
  if(!stream)throw std::runtime_error("output write failed");
  return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
