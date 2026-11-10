function report=cdc_research_selftest(overrides)
% This selftest runs on the user's MATLAB/Octave. No full grid sweep.
if nargin<1,overrides=struct();end
c=cdc_research_options();f=fieldnames(overrides);for j=1:numel(f),assert(isfield(c,f{j}),'Unknown option');c.(f{j})=overrides.(f{j});end
c.speed_kmh=50;c.ux=50/3.6;c.mu=.9;c.steer_deg=3;c.delta=deg2rad([3;3;0;0]);c.trace_stride=0;
[Q,T]=cdc_fullcar_model('prepare',c,[25 25 50 50]);assert(T.root_found&&T.usable,'Trim must be a checked stable equilibrium');
old=rng;restore=onCleanup(@()rng(old));rng(20261111,'twister');
X=repmat(T.x,1,16);X(1,:)=X(1,:)+randn(1,16);X(2,:)=X(2,:)+.2*randn(1,16);X=cdc_fullcar_model('lift',X(1:2,:)',Q,c,T,[0 0 0 0]);
a=cdc_fullcar_model('rhs',X,Q);b=cdc_fullcar_model('scalar_rhs',X,Q);
relative=max(abs(a(:)-b(:))./max(1,abs(b(:))));assert(relative<1e-10,'Scalar/vector vehicle equation mismatch');
p=X(1:2,:)';h=cdc_handling_reference(p,Q);hv=1e-5;hr=1e-6;
p1=p;p1(:,1)=p1(:,1)+hv;p2=p;p2(:,1)=p2(:,1)-hv;jv=(field(p1,Q)-field(p2,Q))/(2*hv);
p1=p;p1(:,2)=p1(:,2)+hr;p2=p;p2(:,2)=p2(:,2)-hr;jr=(field(p1,Q)-field(p2,Q))/(2*hr);
tr=jv(:,1)+jr(:,2);detA=jv(:,1).*jr(:,2)-jr(:,1).*jv(:,2);
traceError=max(abs(tr-h.trace));detError=max(abs(detA-h.determinant));assert(traceError<1e-5&&detError<1e-3,'Analytic reference Jacobian mismatch');
% Checked trim must stay inside the reference and recover.
h0=cdc_handling_reference(T.x(1:2)',Q);assert(h0.inside,'Trim outside H');
t=c;t.horizons=.1;t.terminal_window=.05;t.engine='mcode';r=cdc_research_simulate(T.x,Q,t,T);assert(r.handling_preserved_recoverable==1,'Checked trim not retained');
% Physically bounded predicate relationships.
assert(all(r.handling_preserved_recoverable(:)<=r.recoverable(:)),'Final not subset of recovery');
report=struct('passed',true,'scalar_vector_rhs_relative_error',relative,'handling_trace_error',traceError,'handling_determinant_error',detError,'native_compared',false);
if exist(c.native_executable,'file')==2
 t.horizons=.05;t.terminal_window=.025;x=X(:,1:4);
 ra=cdc_research_simulate(x,Q,t,T);t.engine='native';rb=cdc_research_simulate(x,Q,t,T);
 names=cdc_research_fields();e=0;
 for j=1:numel(names)
  aa=ra.(names{j});bb=rb.(names{j});assert(isequal(isfinite(aa),isfinite(bb)),'Native finite-status mismatch');
  k=isfinite(aa)&isfinite(bb);if any(k(:)),e=max(e,max(abs(aa(k)-bb(k))./max(1,abs(aa(k)))));end
 end
 assert(e<1e-7,'Native/mcode mismatch');report.native_compared=true;report.native_relative_error=e;
end
disp(report);
end
function f=field(p,Q)
X=zeros(32,size(p,1));X(1:2,:)=p';[~,a,~]=cdc_fullcar_model('slips',X,Q);
fz=repmat(Q.Fz_static,1,size(p,1));k=.001635-.0025297*(fz-Q.Fz0_vec)./Q.Fz0_vec;
[fx,fy]=cdc_fullcar_model('tires',k,a,fz,Q);fxb=fx.*Q.cdelta-fy.*Q.sdelta;fyb=fx.*Q.sdelta+fy.*Q.cdelta;
f=[sum(fyb,1)/Q.m-Q.ux*X(2,:);sum(Q.xw.*fyb-Q.yw.*fxb,1)/Q.Iz]';
end
