#pragma once
// Independent MF6.2 steady-state equations; not the proprietary MF-Swift solver.
// Input alpha is wheel lateral/longitudinal velocity atan2, forward travel.
// SI units: kappa[-], alpha/gamma[rad], Fz[N], vx[m/s], pressure[Pa], road_mu[-].
// road_mu directly multiplies LMUX/LMUY; no undocumented 0.9 normalization.
// SIDE=+1 left (default), -1 right mirrors alpha/gamma and Fy/Mz.
// Load coeff once, set SIDE, then prepare() before concurrent evaluations.
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
namespace hmc {
struct TireParameters {
    double SIDE = 1.0;
    double FNOMIN = 0.0;
    double LFZO = 1.0;
    double NOMPRES = 0.0;
    double INFLPRES = 0.0;
    double UNLOADED_RADIUS = 0.0;
    double LONGVL = 16.7;
    double VERTICAL_STIFFNESS = 0.0;
    double LONGITUDINAL_STIFFNESS = 0.0;
    double LATERAL_STIFFNESS = 0.0;
    double Q_RE0 = 1.056;
    double Q_V1 = 0.0009072;
    double BREFF = 20.0;
    double DREFF = 0.291;
    double FREFF = 0.10094;
    double LCX = 1.0;
    double LMUX = 1.0;
    double LEX = 1.0;
    double LKX = 1.0;
    double LHX = 1.0;
    double LVX = 1.0;
    double LCY = 1.0;
    double LMUY = 1.0;
    double LEY = 1.0;
    double LKY = 1.0;
    double LKYC = 1.0;
    double LKZC = 1.0;
    double LHY = 1.0;
    double LVY = 1.0;
    double LTR = 1.0;
    double LRES = 1.0;
    double LXAL = 1.0;
    double LYKA = 1.0;
    double LVYKA = 1.0;
    double LS = 1.0;
    double LMY = 1.0;
    double PCX1 = 1.647;
    double PDX1 = 1.145;
    double PDX2 = -0.2106;
    double PDX3 = 0.0;
    double PEX1 = -0.02192;
    double PEX2 = 0.23613;
    double PEX3 = 0.0;
    double PEX4 = 0.0;
    double PKX1 = 24.125;
    double PKX2 = 15.0;
    double PKX3 = -0.4832;
    double PHX1 = -0.001635;
    double PHX2 = 0.0025297;
    double PVX1 = 0.0;
    double PVX2 = 0.0;
    double PPX1 = -0.04186;
    double PPX2 = 0.0010535;
    double PPX3 = 1.0;
    double PPX4 = 0.015443;
    double RBX1 = 15.488;
    double RBX2 = 47.0;
    double RBX3 = 0.0;
    double RCX1 = 0.5;
    double REX1 = -10.0;
    double REX2 = -2.7775;
    double RHX1 = -0.03;
    double PCY1 = 1.5662;
    double PDY1 = 1.0278;
    double PDY2 = -0.17125;
    double PDY3 = 0.0;
    double PEY1 = -1.1357;
    double PEY2 = -1.0544;
    double PEY3 = -0.008252;
    double PEY4 = -8.844;
    double PEY5 = 0.0;
    double PKY1 = -25.85;
    double PKY2 = 2.1377;
    double PKY3 = -0.7136;
    double PKY4 = 2.0;
    double PKY5 = 0.0;
    double PKY6 = -1.0395;
    double PKY7 = -0.4144;
    double PHY1 = 0.0008675;
    double PHY2 = 7.046e-05;
    double PVY1 = -0.007239;
    double PVY2 = -7.442e-05;
    double PVY3 = -0.15256;
    double PVY4 = -1.0462;
    double PPY1 = -2.0;
    double PPY2 = 2.0;
    double PPY3 = 1.0;
    double PPY4 = 0.07706;
    double PPY5 = 0.0;
    double RBY1 = 20.766;
    double RBY2 = 33.985;
    double RBY3 = -0.04488;
    double RBY4 = 0.0;
    double RCY1 = 1.0796;
    double REY1 = 0.5598;
    double REY2 = 0.011057;
    double RHY1 = 0.01798;
    double RHY2 = 0.006714;
    double RVY1 = -0.010112;
    double RVY2 = -0.0043125;
    double RVY3 = 1.7094;
    double RVY4 = 99.65;
    double RVY5 = 1.8;
    double RVY6 = 5.0;
    double QSY1 = 0.01;
    double QSY2 = 0.0;
    double QSY3 = 0.0004;
    double QSY4 = 4e-05;
    double QSY5 = 0.0;
    double QSY6 = 0.0;
    double QSY7 = 0.85;
    double QSY8 = -0.4;
    double QBZ1 = 12.33;
    double QBZ2 = -5.395;
    double QBZ3 = 0.0;
    double QBZ4 = -0.12843;
    double QBZ5 = -0.1037;
    double QBZ9 = 60.0;
    double QBZ10 = 0.0;
    double QCZ1 = 1.0565;
    double QDZ1 = 0.10777;
    double QDZ2 = -0.015202;
    double QDZ3 = -0.10388;
    double QDZ4 = 0.0;
    double QDZ6 = -0.0004242;
    double QDZ7 = 0.0013088;
    double QDZ8 = -0.18673;
    double QDZ9 = 0.07888;
    double QDZ10 = 0.0;
    double QDZ11 = 0.0;
    double QEZ1 = -10.0;
    double QEZ2 = -4.466;
    double QEZ3 = 0.0;
    double QEZ4 = 0.5128;
    double QEZ5 = -3.5286;
    double QHZ1 = 0.0006972;
    double QHZ2 = 0.00035224;
    double QHZ3 = 0.1225;
    double QHZ4 = -0.11936;
    double PPZ1 = 12.0;
    double PPZ2 = 0.0;
    double SSZ1 = -0.019454;
    double SSZ2 = 0.03583;
    double SSZ3 = -5.0;
    double SSZ4 = -5.0;
};
struct Tire {
    std::unordered_map<std::string,double> coeff;
    TireParameters p;
    bool prepared=false;
    double get(const std::string& key,double fallback=0.0) const {
        auto it=coeff.find(key); return it==coeff.end()?fallback:it->second;
    }
    void prepare() {
        p.SIDE=get("SIDE",1.0);
        p.FNOMIN=get("FNOMIN",0.0);
        p.LFZO=get("LFZO",1.0);
        p.NOMPRES=get("NOMPRES",0.0);
        p.INFLPRES=get("INFLPRES",0.0);
        p.UNLOADED_RADIUS=get("UNLOADED_RADIUS",0.0);
        p.LONGVL=get("LONGVL",16.7);
        p.VERTICAL_STIFFNESS=get("VERTICAL_STIFFNESS",0.0);
        p.LONGITUDINAL_STIFFNESS=get("LONGITUDINAL_STIFFNESS",0.0);
        p.LATERAL_STIFFNESS=get("LATERAL_STIFFNESS",0.0);
        p.Q_RE0=get("Q_RE0",1.056);
        p.Q_V1=get("Q_V1",0.0009072);
        p.BREFF=get("BREFF",20.0);
        p.DREFF=get("DREFF",0.291);
        p.FREFF=get("FREFF",0.10094);
        p.LCX=get("LCX",1.0);
        p.LMUX=get("LMUX",1.0);
        p.LEX=get("LEX",1.0);
        p.LKX=get("LKX",1.0);
        p.LHX=get("LHX",1.0);
        p.LVX=get("LVX",1.0);
        p.LCY=get("LCY",1.0);
        p.LMUY=get("LMUY",1.0);
        p.LEY=get("LEY",1.0);
        p.LKY=get("LKY",1.0);
        p.LKYC=get("LKYC",1.0);
        p.LKZC=get("LKZC",1.0);
        p.LHY=get("LHY",1.0);
        p.LVY=get("LVY",1.0);
        p.LTR=get("LTR",1.0);
        p.LRES=get("LRES",1.0);
        p.LXAL=get("LXAL",1.0);
        p.LYKA=get("LYKA",1.0);
        p.LVYKA=get("LVYKA",1.0);
        p.LS=get("LS",1.0);
        p.LMY=get("LMY",1.0);
        p.PCX1=get("PCX1",1.647);
        p.PDX1=get("PDX1",1.145);
        p.PDX2=get("PDX2",-0.2106);
        p.PDX3=get("PDX3",0.0);
        p.PEX1=get("PEX1",-0.02192);
        p.PEX2=get("PEX2",0.23613);
        p.PEX3=get("PEX3",0.0);
        p.PEX4=get("PEX4",0.0);
        p.PKX1=get("PKX1",24.125);
        p.PKX2=get("PKX2",15.0);
        p.PKX3=get("PKX3",-0.4832);
        p.PHX1=get("PHX1",-0.001635);
        p.PHX2=get("PHX2",0.0025297);
        p.PVX1=get("PVX1",0.0);
        p.PVX2=get("PVX2",0.0);
        p.PPX1=get("PPX1",-0.04186);
        p.PPX2=get("PPX2",0.0010535);
        p.PPX3=get("PPX3",1.0);
        p.PPX4=get("PPX4",0.015443);
        p.RBX1=get("RBX1",15.488);
        p.RBX2=get("RBX2",47.0);
        p.RBX3=get("RBX3",0.0);
        p.RCX1=get("RCX1",0.5);
        p.REX1=get("REX1",-10.0);
        p.REX2=get("REX2",-2.7775);
        p.RHX1=get("RHX1",-0.03);
        p.PCY1=get("PCY1",1.5662);
        p.PDY1=get("PDY1",1.0278);
        p.PDY2=get("PDY2",-0.17125);
        p.PDY3=get("PDY3",0.0);
        p.PEY1=get("PEY1",-1.1357);
        p.PEY2=get("PEY2",-1.0544);
        p.PEY3=get("PEY3",-0.008252);
        p.PEY4=get("PEY4",-8.844);
        p.PEY5=get("PEY5",0.0);
        p.PKY1=get("PKY1",-25.85);
        p.PKY2=get("PKY2",2.1377);
        p.PKY3=get("PKY3",-0.7136);
        p.PKY4=get("PKY4",2.0);
        p.PKY5=get("PKY5",0.0);
        p.PKY6=get("PKY6",-1.0395);
        p.PKY7=get("PKY7",-0.4144);
        p.PHY1=get("PHY1",0.0008675);
        p.PHY2=get("PHY2",7.046e-05);
        p.PVY1=get("PVY1",-0.007239);
        p.PVY2=get("PVY2",-7.442e-05);
        p.PVY3=get("PVY3",-0.15256);
        p.PVY4=get("PVY4",-1.0462);
        p.PPY1=get("PPY1",-2.0);
        p.PPY2=get("PPY2",2.0);
        p.PPY3=get("PPY3",1.0);
        p.PPY4=get("PPY4",0.07706);
        p.PPY5=get("PPY5",0.0);
        p.RBY1=get("RBY1",20.766);
        p.RBY2=get("RBY2",33.985);
        p.RBY3=get("RBY3",-0.04488);
        p.RBY4=get("RBY4",0.0);
        p.RCY1=get("RCY1",1.0796);
        p.REY1=get("REY1",0.5598);
        p.REY2=get("REY2",0.011057);
        p.RHY1=get("RHY1",0.01798);
        p.RHY2=get("RHY2",0.006714);
        p.RVY1=get("RVY1",-0.010112);
        p.RVY2=get("RVY2",-0.0043125);
        p.RVY3=get("RVY3",1.7094);
        p.RVY4=get("RVY4",99.65);
        p.RVY5=get("RVY5",1.8);
        p.RVY6=get("RVY6",5.0);
        p.QSY1=get("QSY1",0.01);
        p.QSY2=get("QSY2",0.0);
        p.QSY3=get("QSY3",0.0004);
        p.QSY4=get("QSY4",4e-05);
        p.QSY5=get("QSY5",0.0);
        p.QSY6=get("QSY6",0.0);
        p.QSY7=get("QSY7",0.85);
        p.QSY8=get("QSY8",-0.4);
        p.QBZ1=get("QBZ1",12.33);
        p.QBZ2=get("QBZ2",-5.395);
        p.QBZ3=get("QBZ3",0.0);
        p.QBZ4=get("QBZ4",-0.12843);
        p.QBZ5=get("QBZ5",-0.1037);
        p.QBZ9=get("QBZ9",60.0);
        p.QBZ10=get("QBZ10",0.0);
        p.QCZ1=get("QCZ1",1.0565);
        p.QDZ1=get("QDZ1",0.10777);
        p.QDZ2=get("QDZ2",-0.015202);
        p.QDZ3=get("QDZ3",-0.10388);
        p.QDZ4=get("QDZ4",0.0);
        p.QDZ6=get("QDZ6",-0.0004242);
        p.QDZ7=get("QDZ7",0.0013088);
        p.QDZ8=get("QDZ8",-0.18673);
        p.QDZ9=get("QDZ9",0.07888);
        p.QDZ10=get("QDZ10",0.0);
        p.QDZ11=get("QDZ11",0.0);
        p.QEZ1=get("QEZ1",-10.0);
        p.QEZ2=get("QEZ2",-4.466);
        p.QEZ3=get("QEZ3",0.0);
        p.QEZ4=get("QEZ4",0.5128);
        p.QEZ5=get("QEZ5",-3.5286);
        p.QHZ1=get("QHZ1",0.0006972);
        p.QHZ2=get("QHZ2",0.00035224);
        p.QHZ3=get("QHZ3",0.1225);
        p.QHZ4=get("QHZ4",-0.11936);
        p.PPZ1=get("PPZ1",12.0);
        p.PPZ2=get("PPZ2",0.0);
        p.SSZ1=get("SSZ1",-0.019454);
        p.SSZ2=get("SSZ2",0.03583);
        p.SSZ3=get("SSZ3",-5.0);
        p.SSZ4=get("SSZ4",-5.0);
        if (!(p.FNOMIN>0 && p.UNLOADED_RADIUS>0 && p.NOMPRES>0 &&
              p.VERTICAL_STIFFNESS>0 && p.LONGITUDINAL_STIFFNESS>0 && p.LATERAL_STIFFNESS>0))
            throw std::invalid_argument("Missing positive tire physical parameter");
        if (p.SIDE!=1 && p.SIDE!=-1) throw std::invalid_argument("Tire SIDE must be +1 or -1");
        prepared=true;
    }
};
struct TireOutput {
    double fx=0,fy=0,mz=0,my=0,re=0,cornering_stiffness=0,longitudinal_stiffness=0;
    double sigma_x=0,sigma_y=0,dx=0,dy=0,kappa_zero=0;
    bool valid=false;
};
inline double mf_sign(double x) { return (x>0)-(x<0); }
inline double mf_den(double x) { return std::abs(x)<1e-12 ? (x<0?-1e-12:1e-12) : x; }
inline double mf_wave(double x,double e) { return x-e*(x-std::atan(x)); }
inline double effective_radius(double fz,double omega,const Tire& tire) {
    if (!tire.prepared) throw std::logic_error("Call Tire::prepare before evaluation");
    const auto& p=tire.p;
    double f=std::max(fz,0.0)/(p.FNOMIN*p.LFZO);
    double rn=p.UNLOADED_RADIUS*(p.Q_RE0+p.Q_V1*std::pow(omega*p.UNLOADED_RADIUS/mf_den(p.LONGVL),2));
    return rn-(p.FNOMIN*p.LFZO/p.VERTICAL_STIFFNESS)*(p.DREFF*std::atan(p.BREFF*f)+p.FREFF*f);
}
inline TireOutput tire_force(double k,double a,double fz,double gamma,double vx,double pressure,
                             const Tire& tire,double road_mu) {
    if (!tire.prepared) throw std::logic_error("Call Tire::prepare before evaluation");
    const auto& p=tire.p;
    TireOutput out;
    if (!(std::isfinite(k)&&std::isfinite(a)&&std::isfinite(fz)&&std::isfinite(gamma)&&
          std::isfinite(vx)&&std::isfinite(pressure)&&std::isfinite(road_mu)) || pressure<=0 || road_mu<0)
        return out;
    const double r0=p.UNLOADED_RADIUS, f0=p.FNOMIN*p.LFZO;
    out.re=effective_radius(std::max(fz,0.0),vx/r0,tire);
    out.valid=true;
    if (fz<=0 || road_mu==0) return out;
    // No silent clamp to TIR validity limits. Caller records domain violations.
    const double side=p.SIDE, direction=vx<0?-1.0:1.0;
    const double ast=std::tan(side*a)*direction, gst=std::sin(side*gamma);
    const double df=(fz-f0)/f0, dp=(pressure-p.NOMPRES)/p.NOMPRES;
    const double mux=p.LMUX*road_mu, muy=p.LMUY*road_mu;
    const double cx=p.PCX1*p.LCX;
    const double mu_x=(p.PDX1+p.PDX2*df)*(1+p.PPX3*dp+p.PPX4*dp*dp)*(1-p.PDX3*gst*gst)*mux;
    const double dx=mu_x*fz;
    const double kx=fz*(p.PKX1+p.PKX2*df)*std::exp(p.PKX3*df)*(1+p.PPX1*dp+p.PPX2*dp*dp)*p.LKX;
    const double shx=(p.PHX1+p.PHX2*df)*p.LHX;
    const double svx=fz*(p.PVX1+p.PVX2*df)*p.LVX*mux;
    const double kxp=k+shx;
    const double ex=std::min((p.PEX1+p.PEX2*df+p.PEX3*df*df)*(1-p.PEX4*mf_sign(kxp))*p.LEX,1.0);
    const double bx=kx/mf_den(cx*dx);
    const double fx0=dx*std::sin(cx*std::atan(mf_wave(bx*kxp,ex)))+svx;
    const double bxa=(p.RBX1+p.RBX3*gst*gst)*std::cos(std::atan(p.RBX2*k))*p.LXAL;
    const double exa=std::min(p.REX1+p.REX2*df,1.0);
    const double gxa=std::cos(p.RCX1*std::atan(mf_wave(bxa*(ast+p.RHX1),exa)))/
                     mf_den(std::cos(p.RCX1*std::atan(mf_wave(bxa*p.RHX1,exa))));
    const double fx=fx0*gxa;
    const double cy=p.PCY1*p.LCY;
    const double mu_y=(p.PDY1+p.PDY2*df)*(1+p.PPY3*dp+p.PPY4*dp*dp)*(1-p.PDY3*gst*gst)*muy;
    const double dy=mu_y*fz;
    const double ky=p.PKY1*f0*(1+p.PPY1*dp)*(1-p.PKY3*std::abs(gst))*
        std::sin(p.PKY4*std::atan(fz/mf_den((p.PKY2+p.PKY5*gst*gst)*f0*(1+p.PPY2*dp))))*p.LKY;
    const double kyg=fz*(p.PKY6+p.PKY7*df)*(1+p.PPY5*dp)*p.LKYC;
    const double svyg=fz*(p.PVY3+p.PVY4*df)*gst*p.LKYC*muy;
    const double shy=(p.PHY1+p.PHY2*df)*p.LHY+(kyg*gst-svyg)/mf_den(ky);
    const double svy=fz*(p.PVY1+p.PVY2*df)*p.LVY*muy+svyg;
    const double ay=ast+shy;
    const double ey=std::min((p.PEY1+p.PEY2*df)*(1+p.PEY5*gst*gst-(p.PEY3+p.PEY4*gst)*mf_sign(ay))*p.LEY,1.0);
    const double by=ky/mf_den(cy*dy);
    const double fy0=dy*std::sin(cy*std::atan(mf_wave(by*ay,ey)))+svy;
    const double byk=(p.RBY1+p.RBY4*gst*gst)*std::cos(std::atan(p.RBY2*(ast-p.RBY3)))*p.LYKA;
    const double eyk=std::min(p.REY1+p.REY2*df,1.0), shyk=p.RHY1+p.RHY2*df;
    const double gyk=std::cos(p.RCY1*std::atan(mf_wave(byk*(k+shyk),eyk)))/
                     mf_den(std::cos(p.RCY1*std::atan(mf_wave(byk*shyk,eyk))));
    const double dvyk=mu_y*fz*(p.RVY1+p.RVY2*df+p.RVY3*gst)*std::cos(std::atan(p.RVY4*ast));
    const double svyk=dvyk*std::sin(p.RVY5*std::atan(p.RVY6*k))*p.LVYKA;
    const double fy=fy0*gyk+svyk;
    // MF6.2 pneumatic trail + residual + Fx lever arm, no turn-slip.
    const double sht=p.QHZ1+p.QHZ2*df+(p.QHZ3+p.QHZ4*df)*gst;
    const double at=ast+sht, ar=ast+shy+svy/mf_den(ky);
    const double bt=(p.QBZ1+p.QBZ2*df+p.QBZ3*df*df)*(1+p.QBZ4*gst+p.QBZ5*std::abs(gst))*p.LKY/mf_den(muy);
    const double ct=p.QCZ1;
    const double dt=(p.QDZ1+p.QDZ2*df)*(1-p.PPZ1*dp)*fz*r0/f0*p.LTR*
                    (1+p.QDZ3*std::abs(gst)+p.QDZ4*gst*gst)*direction;
    const double et=std::min((p.QEZ1+p.QEZ2*df+p.QEZ3*df*df)*
                    (1+(p.QEZ4+p.QEZ5*gst)*(2.0/3.14159265358979323846)*std::atan(bt*ct*at)),1.0);
    const double kr=kx/mf_den(ky);
    const double ateq=std::atan(std::hypot(std::tan(at),kr*k))*mf_sign(at);
    const double areq=std::atan(std::hypot(std::tan(ar),kr*k))*mf_sign(ar);
    const double trail=dt*std::cos(ct*std::atan(mf_wave(bt*ateq,et)))*std::cos(a);
    const double br=p.QBZ9*p.LKY/mf_den(muy)+p.QBZ10*by*cy;
    const double dr=fz*r0*((p.QDZ6+p.QDZ7*df)*p.LRES+
                    (p.QDZ8+p.QDZ9*df)*(1+p.PPZ2*dp)*gst*p.LKZC+
                    (p.QDZ10+p.QDZ11*df)*gst*std::abs(gst))*muy*direction*std::cos(a);
    const double mzr=dr*std::cos(std::atan(br*areq));
    const double lever=r0*(p.SSZ1+p.SSZ2*fy/f0+(p.SSZ3+p.SSZ4*df)*gst)*p.LS;
    const double mz=-trail*(fy-svyk)+mzr+lever*fx;
    const double vnorm=std::abs(vx/mf_den(p.LONGVL));
    const double my=-r0*f0*(p.QSY1+p.QSY2*fx/f0+p.QSY3*vnorm+p.QSY4*std::pow(vnorm,4)+
                    (p.QSY5+p.QSY6*fz/f0)*gst*gst)*std::pow(fz/f0,p.QSY7)*
                    std::pow(pressure/p.NOMPRES,p.QSY8)*p.LMY*mf_sign(vx);
    out.fx=fx; out.fy=side*fy; out.mz=side*mz; out.my=my;
    out.cornering_stiffness=ky;out.longitudinal_stiffness=kx;
    out.sigma_x=std::abs(kx)/p.LONGITUDINAL_STIFFNESS;
    out.sigma_y=std::abs(ky)/p.LATERAL_STIFFNESS;
    out.dx=dx;out.dy=dy;out.kappa_zero=-shx;
    out.valid=std::isfinite(out.fx)&&std::isfinite(out.fy)&&std::isfinite(out.mz)&&std::isfinite(out.my)&&out.re>0;
    return out;
}
} // namespace hmc
