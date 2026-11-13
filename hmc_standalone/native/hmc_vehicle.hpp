#pragma once
// Independent HMC numerical model. No CarSim/TNO library is loaded.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include "hmc_tire.hpp"
namespace hmc {
constexpr double pi=3.141592653589793238462643383279502884;
constexpr double rad=pi/180.0;
struct Json {
 enum Kind {Null,Number,Boolean,String,Array,Object};Kind kind=Null;
 double number=0;bool boolean=false;std::string string;
 std::vector<Json> array;std::map<std::string,Json> object;
 bool has(const std::string& k)const{return object.find(k)!=object.end();}
 const Json& at(const std::string& k)const{auto it=object.find(k);if(it==object.end())throw std::runtime_error("Missing JSON field "+k);return it->second;}
 double num(double d=0)const{return kind==Number?number:(kind==Boolean?(boolean?1:0):d);}
 double num(const std::string&k,double d=0)const{return has(k)?at(k).num(d):d;}
 std::string str(const std::string&d="")const{return kind==String?string:d;}
 void flatten(std::vector<double>&v)const{if(kind==Array){for(const auto&x:array)x.flatten(v);}else if(kind==Number||kind==Boolean)v.push_back(num());}
 std::vector<double> vec()const{std::vector<double>v;flatten(v);return v;}
 std::vector<double> vec(const std::string&k)const{return at(k).vec();}
 static Json parse(const std::string& text){std::size_t i=0;Json v=parse_value(text,i);space(text,i);if(i!=text.size())throw std::runtime_error("Trailing JSON data");return v;}
 static void space(const std::string&s,std::size_t&i){while(i<s.size()&&(s[i]==' '||s[i]=='\n'||s[i]=='\r'||s[i]=='\t'))++i;}
 static unsigned hex(char c){if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;throw std::runtime_error("Invalid JSON unicode");}
 static std::string parse_string(const std::string&s,std::size_t&i){
  if(i>=s.size()||s[i++]!='"')throw std::runtime_error("Expected JSON string");std::string o;
  while(i<s.size()){
   char c=s[i++];if(c=='"')return o;if(c!='\\'){o+=c;continue;}
   if(i>=s.size())break;c=s[i++];switch(c){case '"':o+='"';break;case '\\':o+='\\';break;case '/':o+='/';break;case 'b':o+='\b';break;case 'f':o+='\f';break;case 'n':o+='\n';break;case 'r':o+='\r';break;case 't':o+='\t';break;
    case 'u':{if(i+4>s.size())throw std::runtime_error("Short JSON unicode");unsigned cp=0;for(int k=0;k<4;++k)cp=16*cp+hex(s[i++]);if(cp<0x80)o+=char(cp);else if(cp<0x800){o+=char(0xc0|(cp>>6));o+=char(0x80|(cp&63));}else{o+=char(0xe0|(cp>>12));o+=char(0x80|((cp>>6)&63));o+=char(0x80|(cp&63));}break;}
    default:throw std::runtime_error("Invalid JSON escape");}
  }throw std::runtime_error("Unterminated JSON string");
 }
 static Json parse_value(const std::string&s,std::size_t&i){
  space(s,i);if(i>=s.size())throw std::runtime_error("Unexpected end of JSON");Json v;
  if(s[i]=='{'){v.kind=Object;++i;space(s,i);if(i<s.size()&&s[i]=='}'){++i;return v;}while(true){space(s,i);auto k=parse_string(s,i);space(s,i);if(i>=s.size()||s[i++]!=':')throw std::runtime_error("Expected JSON colon");v.object.emplace(k,parse_value(s,i));space(s,i);if(i>=s.size())break;char c=s[i++];if(c=='}')return v;if(c!=',')throw std::runtime_error("Expected JSON object separator");}throw std::runtime_error("Unterminated JSON object");}
  if(s[i]=='['){v.kind=Array;++i;space(s,i);if(i<s.size()&&s[i]==']'){++i;return v;}while(true){v.array.push_back(parse_value(s,i));space(s,i);if(i>=s.size())break;char c=s[i++];if(c==']')return v;if(c!=',')throw std::runtime_error("Expected JSON array separator");}throw std::runtime_error("Unterminated JSON array");}
  if(s[i]=='"'){v.kind=String;v.string=parse_string(s,i);return v;}
  if(s.compare(i,4,"true")==0){i+=4;v.kind=Boolean;v.boolean=true;return v;}
  if(s.compare(i,5,"false")==0){i+=5;v.kind=Boolean;return v;}
  if(s.compare(i,4,"null")==0){i+=4;return v;}
  std::size_t used=0;v.kind=Number;v.number=std::stod(s.substr(i),&used);if(!used||!std::isfinite(v.number))throw std::runtime_error("Invalid JSON number");i+=used;return v;
 }
};
using State=std::array<double,34>;
struct Observation {std::array<double,4> fz{},alpha{},kappa{},uw{},steer{},camber{};double ay=0,ax=0;};
inline std::array<double,4> arr4(const Json&j,const std::string&key,double fallback=0){std::array<double,4>a;auto v=j.has(key)?j.vec(key):std::vector<double>{fallback};if(v.size()==1)a.fill(v[0]);else{if(v.size()!=4)throw std::runtime_error("Expected four values: "+key);std::copy(v.begin(),v.end(),a.begin());}return a;}
inline double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
inline double interpolate(double x,const std::vector<double>&axis,const std::vector<double>&y){if(axis.empty()||axis.size()!=y.size())throw std::runtime_error("Invalid interpolation data");if(x<=axis.front())return y.front();if(x>=axis.back())return y.back();auto it=std::upper_bound(axis.begin(),axis.end(),x);std::size_t i=(it-axis.begin())-1;double a=(x-axis[i])/(axis[i+1]-axis[i]);return y[i]+a*(y[i+1]-y[i]);}
}
namespace hmc {
struct Table {
 std::vector<double>x,y;
 Table()=default;
 Table(const Json&j,const std::string&xkey,const std::string&ykey):x(j.vec(xkey)),y(j.vec(ykey)){if(x.size()<2||x.size()!=y.size())throw std::runtime_error("Invalid vehicle table");for(std::size_t i=1;i<x.size();++i)if(!(x[i]>x[i-1]))throw std::runtime_error("Vehicle table axis must increase");}
 double operator()(double q,bool bounded=false)const{if(bounded)q=clamp(q,x.front(),x.back());auto it=std::upper_bound(x.begin()+1,x.end()-1,q);std::size_t j=it-x.begin()-1;double a=(q-x[j])/(x[j+1]-x[j]);return y[j]+a*(y[j+1]-y[j]);}
};
struct Vehicle {
 double m,ms,g,Ixx,Iyy,Iz_total,hcg,tf,tr,ux_target,mu,steer_deg,steering_ratio,gear_rot_ratio;
 double Karb_f,Karb_r,Carb_f,Carb_r,rho_air,aero_area,Cx,KP,KI,KP3,max_power;
 bool compliance;
 std::array<double,4>mu_u,Iw,kt,ct,Fs0,Fz0,pressure,Re0,xw,yw,side,camber0,toe0,jounce0,spring0,mode_rate,drive_split;
 std::array<double,4>CT_FX,CS_FY,CS_MZ,CC_FX,CI_FY,CI_MZ;
 std::array<Tire,4>tires;
 std::array<Table,4>spring_comp,spring_ext,jounce_stop,rebound_stop,camber,toe,susp_x,susp_lat,steering,damper;
 static void collect_tires(const Json&j,std::vector<const Json*>&out){if(j.kind==Json::Object)out.push_back(&j);else if(j.kind==Json::Array)for(const auto&v:j.array)collect_tires(v,out);}
 Vehicle(const Json&q){
  m=q.at("m").num();ms=q.at("ms").num();g=q.at("g").num();Ixx=q.at("Ixx").num();Iyy=q.at("Iyy").num();Iz_total=q.at("Iz_total").num();hcg=q.at("h_cg_sprung").num();tf=q.at("tf").num();tr=q.at("tr").num();ux_target=q.at("ux_target").num();mu=q.at("mu").num();steer_deg=q.at("steer_deg").num();steering_ratio=q.at("steering_ratio").num();gear_rot_ratio=q.at("gear_rot_ratio").num();
  Karb_f=q.at("Karb_f").num();Karb_r=q.at("Karb_r").num();Carb_f=q.at("Carb_f").num();Carb_r=q.at("Carb_r").num();rho_air=q.at("rho_air").num();aero_area=q.at("aero_area_m2").num();Cx=q.at("aero_Cx").num();KP=q.at("speed_KP").num();KI=q.at("speed_KI").num();KP3=q.at("speed_KP3").num();max_power=q.at("max_power_W").num();compliance=q.num("include_compliance",1)!=0;
  mu_u=arr4(q,"mu_u");Iw=arr4(q,"Iw");kt=arr4(q,"kt");ct=arr4(q,"ct");Fs0=arr4(q,"Fs0");Fz0=arr4(q,"Fz0");pressure=arr4(q,"pressure");Re0=arr4(q,"Re0");xw=arr4(q,"xw");yw=arr4(q,"yw");side=arr4(q,"side");camber0=arr4(q,"camber_static_rad");toe0=arr4(q,"toe_static_rad");jounce0=arr4(q,"jounce0");spring0=arr4(q,"spring_compression0");mode_rate=arr4(q,"spring_mode_rate");drive_split=arr4(q,"drive_split");
  const auto&cc=q.at("compliance");CT_FX=arr4(cc.at("CT_FX"),"SI");CS_FY=arr4(cc.at("CS_FY"),"SI");CS_MZ=arr4(cc.at("CS_MZ"),"SI");CC_FX=arr4(cc.at("CC_FX"),"SI");CI_FY=arr4(cc.at("CI_FY"),"SI");CI_MZ=arr4(cc.at("CI_MZ"),"SI");
  std::vector<const Json*> tj;collect_tires(q.at("tires"),tj);if(tj.size()!=4)throw std::runtime_error("Expected four tire dictionaries");
  const auto&tables=q.at("tables");const std::string names[4]={"FL","FR","RL","RR"};auto va=q.vec("vaxis"),dm=q.vec("damper_map");if(dm.size()!=4*va.size())throw std::runtime_error("Invalid damper matrix");
  for(int i=0;i<4;++i){for(const auto&entry:tj[i]->object)if(entry.second.kind==Json::Number||entry.second.kind==Json::Boolean)tires[i].coeff[entry.first]=entry.second.num();tires[i].prepare();
   const auto&n=names[i];spring_comp[i]=Table(tables.at("spring_comp").at(n),"x_m","y_N");spring_ext[i]=Table(tables.at("spring_ext").at(n),"x_m","y_N");jounce_stop[i]=Table(tables.at("jounce_stop").at(n),"x_m","y_N");rebound_stop[i]=Table(tables.at("rebound_stop").at(n),"x_m","y_N");camber[i]=Table(tables.at("camber").at(n),"x_m","y_rad");toe[i]=Table(tables.at("toe").at(n),"x_m","y_rad");susp_x[i]=Table(tables.at("susp_x").at(n),"x_m","y_m");susp_lat[i]=Table(tables.at("susp_lat").at(n),"x_m","y_m");if(i<2)steering[i]=Table(tables.at("steering").at(n),"x_rad","y_rad");damper[i].x=va;damper[i].y.assign(dm.begin()+i*va.size(),dm.begin()+(i+1)*va.size());
  }
 }
 double body_height(const State&x,int i)const{return x[2]+yw[i]*x[3]-xw[i]*x[4];}
 double body_velocity(const State&x,int i)const{return x[9]+yw[i]*x[10]-xw[i]*x[11];}
 double radius(double fz,double omega,int i)const{const auto&p=tires[i].p;double r=p.UNLOADED_RADIUS*(p.Q_RE0+p.Q_V1*std::pow(omega*p.UNLOADED_RADIUS/p.LONGVL,2))-(p.FNOMIN/p.VERTICAL_STIFFNESS)*(p.DREFF*std::atan(p.BREFF*fz/p.FNOMIN)+p.FREFF*fz/p.FNOMIN);return std::max(r,.1*p.UNLOADED_RADIUS);}
 void rhs(const State&x,State&dx,Observation*obs)const{
  std::array<double,4>raw,fz,jounce,delta,gamma,xpos,ypos,fx,fy,mz,my,sx,sy,uw,vw,re,kap,alpha,fs,fd,df;
  const double gear=steer_deg*steering_ratio*rad*gear_rot_ratio;
  for(int i=0;i<4;++i){raw[i]=Fz0[i]-kt[i]*x[5+i]-ct[i]*x[12+i];fz[i]=std::max(raw[i],0.0);jounce[i]=jounce0[i]+x[5+i]-body_height(x,i);double bt=toe0[i]+toe[i](jounce[i]),bc=camber0[i]+camber[i](jounce[i]),command=i<2?steering[i](gear,true):0;delta[i]=command-side[i]*bt;gamma[i]=-side[i]*bc+x[3];xpos[i]=xw[i]+susp_x[i](jounce[i]);ypos[i]=yw[i]-side[i]*susp_lat[i](jounce[i]);}
  if(compliance){auto bd=delta,bg=gamma;for(int it=0;it<2;++it)for(int i=0;i<4;++i){double ub=x[32]-ypos[i]*x[1],vb=x[0]+xpos[i]*x[1],u=std::cos(delta[i])*ub+std::sin(delta[i])*vb;auto t=tire_force(x[16+i],x[20+i],fz[i],gamma[i],u,pressure[i],tires[i],mu);delta[i]=bd[i]-side[i]*CT_FX[i]*t.fx+CS_FY[i]*t.fy+CS_MZ[i]*t.mz;gamma[i]=bg[i]+side[i]*CC_FX[i]*t.fx+CI_FY[i]*t.fy+CI_MZ[i]*t.mz;}}
  double sumfx=0,sumfy=0,moment=0;
  for(int i=0;i<4;++i){double ub=x[32]-ypos[i]*x[1],vb=x[0]+xpos[i]*x[1],cd=std::cos(delta[i]),sd=std::sin(delta[i]);uw[i]=cd*ub+sd*vb;vw[i]=-sd*ub+cd*vb;alpha[i]=std::atan2(vw[i],uw[i]);re[i]=radius(fz[i],x[24+i],i);kap[i]=(re[i]*x[24+i]-uw[i])/std::max(std::abs(uw[i]),.5);auto t=tire_force(x[16+i],x[20+i],fz[i],gamma[i],uw[i],pressure[i],tires[i],mu);fx[i]=t.fx;fy[i]=t.fy;mz[i]=t.mz;my[i]=t.my;sx[i]=std::max(t.sigma_x,.001);sy[i]=std::max(t.sigma_y,.001);double fxbody=fx[i]*cd-fy[i]*sd,fybody=fx[i]*sd+fy[i]*cd;sumfx+=fxbody;sumfy+=fybody;moment+=xpos[i]*fybody-ypos[i]*fxbody+mz[i];
   double rate=x[12+i]-body_velocity(x,i),comp=spring0[i]+jounce[i]-jounce0[i];double force=rate<0?spring_ext[i](comp):spring_comp[i](comp);force+=(mode_rate[i]-30790)*comp;force+=jounce_stop[i](jounce[i],false)+rebound_stop[i](jounce[i],false);fd[i]=damper[i](1000*rate,true);fs[i]=force+fd[i];
  }
  double mf=-Karb_f*(x[3]-(x[5]-x[6])/tf)-Carb_f*(x[10]-(x[12]-x[13])/tf),mr=-Karb_r*(x[3]-(x[7]-x[8])/tr)-Carb_r*(x[10]-(x[14]-x[15])/tr);fs[0]+=mf/tf;fs[1]-=mf/tf;fs[2]+=mr/tr;fs[3]-=mr/tr;
  double ay=sumfy/m,drag=.5*rho_air*aero_area*Cx*x[32]*std::abs(x[32]),ax=(sumfx-drag)/m,sumdf=0,roll=0,pitch=0;
  for(int i=0;i<4;++i){df[i]=fs[i]-Fs0[i];sumdf+=df[i];roll+=yw[i]*df[i];pitch+=xw[i]*df[i];}
  dx.fill(0);dx[0]=ay-x[32]*x[1];dx[1]=moment/Iz_total;
  for(int i=0;i<7;++i)dx[2+i]=x[9+i];dx[9]=sumdf/ms;dx[10]=(roll+ms*hcg*ay)/Ixx;dx[11]=(-pitch-ms*hcg*ax)/Iyy;
  double error=ux_target-x[32],demand=KP*error+KP3*error*error*error+KI*x[33],throttle=clamp(demand,0,1),drive=throttle*max_power/std::max(std::abs(x[32]),1.0);
  for(int i=0;i<4;++i){dx[12+i]=(raw[i]-Fz0[i]-df[i])/mu_u[i];double speed=std::max(std::abs(uw[i]),.5);dx[16+i]=speed/sx[i]*(kap[i]-x[16+i]);dx[20+i]=speed/sy[i]*(alpha[i]-x[20+i]);double torque=drive_split[i]*drive*re[i];dx[24+i]=(torque-re[i]*fx[i]+my[i])/Iw[i];}
  dx[28]=ay-x[28];dx[29]=x[1];dx[30]=x[32]*std::cos(x[29])-x[0]*std::sin(x[29]);dx[31]=x[32]*std::sin(x[29])+x[0]*std::cos(x[29]);dx[32]=ax+x[1]*x[0];dx[33]=((demand<=0&&error<0)||(demand>=1&&error>0))?0:error;
  if(obs){obs->fz=raw;obs->ay=ay;obs->ax=ax;obs->alpha=alpha;obs->kappa=kap;obs->uw=uw;obs->steer=delta;obs->camber=gamma;}
 }
};
}
