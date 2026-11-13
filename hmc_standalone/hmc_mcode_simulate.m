function out=hmc_mcode_simulate(X,Q,T,c,wear)
%HMC_MCODE_SIMULATE Independent RK4 reference with full trajectory policy.
% Small batches retain every time sample, including the initial transient.
% The native engine is preferable for the full sweep.
n=size(X,2);h=c.horizons;steps=round(max(h)/c.dt);t=(0:steps)*c.dt;
assert(all(abs(h/c.dt-round(h/c.dt))<1e-8),'Horizon not on integration grid');
out=struct();trace={};
for first=1:c.batch_size:n
 ids=first:min(n,first+c.batch_size-1);nb=numel(ids);
 states=zeros(34,steps+1,nb);loads=zeros(4,steps+1,nb);ay=zeros(steps+1,nb);
 x=X(:,ids);states(:,1,:)=reshape(x,34,1,nb);
 for j=1:steps+1
  [a,O]=hmc_vehicle_model('rhs',x,Q);
  if isfield(O,'Fz_raw'),fz=O.Fz_raw;else,fz=O.Fz;end
  loads(:,j,:)=reshape(fz,4,1,nb);ay(j,:)=O.ay;
  if j>steps,break;end
  b=hmc_vehicle_model('rhs',x+.5*c.dt*a,Q);
  d=hmc_vehicle_model('rhs',x+.5*c.dt*b,Q);
  e=hmc_vehicle_model('rhs',x+c.dt*d,Q);
  x=x+c.dt/6*(a+2*b+2*d+e);
  states(:,j+1,:)=reshape(x,34,1,nb);
 end
 for k=1:nb
  for ih=1:numel(h)
   take=t<=h(ih)+1e-8;cp=c;cp.horizons=h(ih);cp.horizons_s=h(ih);
   s=hmc_classify(t(take),states(:,take,k),struct('Fz_raw',loads(:,take,k),'ay',ay(take,k)),T,cp,wear);
   names=fieldnames(s);
   for q=1:numel(names)
    key=names{q};value=s.(key);
    if (isnumeric(value)||islogical(value))&&isscalar(value)
     if ~isfield(out,key),out.(key)=nan(n,numel(h));end
     out.(key)(ids(k),ih)=value;
    end
   end
  end
  if c.trace_stride>0
   keep=unique([1:c.trace_stride:numel(t),numel(t)]);
   trace{ids(k)}=struct('time_s',t(keep),'state',states(:,keep,k), ...
    'ay_mps2',ay(keep,k),'Fz_N',loads(:,keep,k));
  end
 end
 if ~isfield(out,'final_state'),out.final_state=nan(n,34);end
 out.final_state(ids,:)=x';
end
out.engine='independent_mcode';out.trace=trace;
end
