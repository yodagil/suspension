function bits=cdc_research_codes(X,dx,O,Q,c)
% Bit 1 yaw POLICY, bit 2 ay POLICY; bits 4..128 end model-domain rollout.
n=size(X,2);bits=zeros(1,n,'uint16');
yaw=abs(X(2,:))>c.yaw_factor*c.mu*Q.g/Q.ux;ay=abs(O.ay)>c.ay_factor*c.mu*Q.g;
ltr=abs(O.LTR)>=c.LTR_limit;contact=any(O.Fz_raw<=0,1);
axle=abs(O.LTRf)>=c.LTR_limit|abs(O.LTRr)>=c.LTR_limit;
num=any(~isfinite(X),1)|any(~isfinite(dx),1)|~isfinite(O.ay)|any(~isfinite(O.Fz_raw),1)|~isfinite(O.LTR)|~isfinite(O.LTRf)|~isfinite(O.LTRr);
guard=abs(X(1,:))>c.guard.vc|abs(X(2,:))>c.guard.r|abs(X(4,:))>c.guard.phi|any(abs(X(17:20,:))>c.guard.kappa+1e-8,1)|any(abs(X(21:24,:))>c.guard.alpha,1)|any(X(25:28,:)<0|X(25:28,:)>c.guard.omega,1)|any(O.uw<=0,1);
clipped=any(O.Fz_raw<=Q.Fz_force_min|O.Fz_raw>=Q.Fz_force_max|O.Fz_elastic_raw<=Q.Fz_force_min|O.Fz_elastic_raw>=Q.Fz_force_max,1);
checks={yaw,ay,ltr,contact,axle,num,guard,clipped&c.reject_force_clipping};
for j=1:8,ids=checks{j};bits(ids)=bitor(bits(ids),uint16(2^(j-1)));end
end
