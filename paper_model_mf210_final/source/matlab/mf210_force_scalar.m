function [Fx,Fy]=mf210_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)
% Exact scalar MF kernel extracted from the validated 210-condition source.
[Fx,Fy]=calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y);
end

function [Fx,Fy] = calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)

%

% The available coefficient set contains pure-slip MF coefficients but not

% the complete MF6.2 combined-slip R-coefficients. Therefore this model uses

% a longitudinal-priority friction ellipse:

%

%   Fx = Fx0

%   Fy = Fy0*sqrt(max(0,1-(Fx0/(mu_x*Fz))^2))

%

% This is specifically chosen because the current comparison shows Fx is

% already close while the pure-slip model overpredicts Fy. Exact CarSim

% agreement requires the TIR RBX/RCX/REX/RBY/RCY/REY/RVY coefficients.

Fx0 = calc_mf_longitudinal_scalar(kappa,Fz,Fz0,wear,mu_scale_x);

Fy0 = calc_mf_lateral_scalar(alpha,Fz,Fz0,wear,mu_scale_y);

mux_peak = calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x);

Fx_cap = mux_peak*Fz;

if Fx_cap < 1.0; Fx_cap=1.0; end

rho_x = abs(Fx0)/Fx_cap;

if rho_x>1.0; rho_x=1.0; end

Gy = sqrt(max(0.0,1.0-rho_x*rho_x));

Fx = Fx0;

Fy = Fy0*Gy;

end

%% ========================================================================

% Longitudinal peak-friction estimate used by friction ellipse

%% ========================================================================

function mux = calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x)

PDX1 = 1.145;

PDX2 = -0.2106;

wear_case = normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = abs(PDX1+PDX2*dfz)*mu_scale_x;

if mux<0.05; mux=0.05; end

end

%% ========================================================================

% Pure longitudinal MF-Swift 6.2 force for one tire

%% ========================================================================

function Fx = calc_mf_longitudinal_scalar(kappa, Fz, Fz0, wear, mu_scale_x)

PCX1 = 1.647;

PDX1 = 1.145;

PDX2 = -0.2106;

PEX1 = -0.02192;

PEX2 = 0.23613;

PEX3 = 0.0;

PEX4 = 0.0;

PKX1 = 24.125;

PKX2 = 15.0;

PKX3 = -0.4832;

PHX1 = -0.001635;

PHX2 =  0.0025297;

PVX1 = 0.0;

PVX2 = 0.0;

wear_case = normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

    PKX1 = PKX1*0.95;

    PKX2 = PKX2*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

    PKX1 = PKX1*0.90;

    PKX2 = PKX2*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = PDX1 + PDX2*dfz;

Dx  = mux*Fz*mu_scale_x;

Cx  = PCX1;

Kx = Fz*(PKX1 + PKX2*dfz)*exp(PKX3*dfz);

if abs(Cx*Dx) < 1.0e-9

    Bx = 0.0;

else

    Bx = Kx/(Cx*Dx);

end

Ex_base = PEX1 + PEX2*dfz + PEX3*dfz^2;

SHx = PHX1 + PHX2*dfz;

SVx = Fz*(PVX1 + PVX2*dfz);

kappa_x = kappa + SHx;

E = Ex_base*(1.0 - PEX4*sign(kappa_x));

if E > 1.0

    E = 1.0;

end

phi_x = Bx*kappa_x;

Fx = Dx*sin(Cx*atan(phi_x - E*(phi_x-atan(phi_x)))) + SVx;

end

%% ========================================================================

% Pure lateral MF-Swift 6.2 force for one tire

%% ========================================================================

function Fy = calc_mf_lateral_scalar(alpha, Fz, Fz0, wear, mu_scale_y)

wear_case = normalize_wear_case(wear);

if wear_case == 0

    PCY1 = 1.5662;

    PDY1 = 1.0278;

    PDY2 = -0.17125;

    PEY1 = -1.1357;

    PEY2 = -1.0544;

    PKY1 = -25.85;

    PKY2 = 2.1377;

    PKY4 = 2.0;

    PHY1 = 8.675e-4;

    PHY2 = 7.046e-5;

    PVY1 = -0.007239;

    PVY2 = -7.442e-5;

elseif wear_case == 25

    PCY1 = 1.5457;

    PDY1 = 1.0286;

    PDY2 = -0.16688;

    PEY1 = -1.439;

    PEY2 = -1.6063;

    PKY1 = -26.0;

    PKY2 = 1.9164;

    PKY4 = 2.0;

    PHY1 = 1.82e-3;

    PHY2 = -7.193e-4;

    PVY1 = 1.802e-3;

    PVY2 = -1.1497e-2;

else

    PCY1 = 1.5057;

    PDY1 = 1.0067;

    PDY2 = -0.2019;

    PEY1 = -2.0;

    PEY2 = -2.0;

    PKY1 = -27.27;

    PKY2 = 1.5963;

    PKY4 = 2.0;

    PHY1 = 2.3735e-3;

    PHY2 = 2.38e-4;

    PVY1 = 0.01487;

    PVY2 = -0.011025;

end

dfz = (Fz-Fz0)/Fz0;

muy = PDY1 + PDY2*dfz;

Dy  = muy*Fz*mu_scale_y;

Cy  = PCY1;

Ky = PKY1*Fz0*sin(PKY4*atan(Fz/(PKY2*Fz0)));

if abs(Cy*Dy) < 1.0e-9

    By = 0.0;

else

    By = Ky/(Cy*Dy);

end

Ey_base = PEY1 + PEY2*dfz;

SHy = PHY1 + PHY2*dfz;

SVy = Fz*(PVY1 + PVY2*dfz);

alpha_y = alpha + SHy;

E = Ey_base;

if E > 1.0

    E = 1.0;

end

phi_y = By*alpha_y;

Fy = Dy*sin(Cy*atan(phi_y - E*(phi_y-atan(phi_y)))) + SVy;

end

%% ========================================================================

% Normalize wear input to 0 / 25 / 50

%% ========================================================================

function wear_case = normalize_wear_case(wear)

if wear < 12.5

    wear_case = 0;

elseif wear < 37.5

    wear_case = 25;

else

    wear_case = 50;

end

end

%% ========================================================================

% Calculate model-based damper rates

%% ========================================================================

