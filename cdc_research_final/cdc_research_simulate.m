function out=cdc_research_simulate(X,Q,c,T)
% Observes EVERY RK4 endpoint. This is a sampled, finite-horizon test.
% Yaw/ay/H-reference exits are recorded without terminating recovery tests.
% Contact/LTR/numerical/model-domain failure terminates the corresponding run.
assert(size(X,1)==32&&all(isfinite(X(:))),'X must be finite 32xN.');
if strcmp(c.engine,'native'),out=cdc_research_native(X,Q,c,T);return;end
assert(strcmp(c.engine,'mcode'),'engine must be mcode or native');
n=size(X,2);names=cdc_research_fields();nh=numel(c.horizons);out=struct();
for j=1:numel(names),out.(names{j})=nan(n,nh);end
out.final_state=zeros(n,32);out.trace=[];
for first=1:c.batch_size:n
 ids=first:min(n,first+c.batch_size-1);b=one_batch(X(:,ids),Q,c,T);
 for j=1:numel(names),out.(names{j})(ids,:)=b.(names{j});end
 out.final_state(ids,:)=b.final_state;
 if ~isempty(b.trace),b.trace(:,1)=b.trace(:,1)+first-1;out.trace=[out.trace;b.trace];end %#ok<AGROW>
end
end

function out=one_batch(X,Q,c,T)
n=size(X,2);dt=c.Ts/c.Nsub;steps=round(c.horizons/dt);nh=numel(steps);nf=numel(cdc_research_fields());
records=nan(n,nh,nf);active=true(1,n);reason=zeros(1,n,'uint16');
kept=true(1,n);prepeak=true(1,n);insideLast=true(1,n);
firstH=inf(1,n);firstAy=firstH;firstYaw=firstH;firstHard=firstH;firstSlope=firstH;
outside=zeros(1,n);minMargin=firstH;minC=firstH;peakR=zeros(1,n);peakAy=peakR;peakLTR=peakR;minFz=firstH;
lastNot=-inf(1,n);err=inf(1,n);derr=err;observed=zeros(1,n);ih=1;trace=zeros(0,13);
I=repmat(c.I_mA(:),1,n);feedback=~isempty(c.controller);
usable=T.usable;if feedback,usable=T.root_found&&T.admissible;end
for k=0:steps(end)
 t=k*dt;ids=find(active);
 if ~isempty(ids)
  if k>0,outside(ids)=outside(ids)+dt*double(~insideLast(ids));end
  xa=X(:,ids);
  if feedback&&mod(k,c.Nsub)==0
   request=c.controller(t,xa,Q);
   assert(isequal(size(request),[4 numel(ids)])&&all(isfinite(request(:))),'controller must return finite 4xN currents');
   request=min(max(request,100),1800);dI=c.current_rate_limit_mA_s*c.Ts;
   I(:,ids)=I(:,ids)+min(max(request-I(:,ids),-dI),dI);
  end
  [k1,O]=cdc_fullcar_model('rhs',xa,Q);
  if feedback,k1=cdc_fullcar_model('rhs_current',xa,Q,I(:,ids));end
  bits=cdc_research_codes(xa,k1,O,Q,c);reason(ids)=bitor(reason(ids),bits);
  H=cdc_handling_reference(xa(1:2,:)',Q);inside=H.inside';
  kept(ids)=kept(ids)&inside;insideLast(ids)=inside;
  hit=ids(~inside);firstH(hit)=min(firstH(hit),t);minMargin(ids)=min(minMargin(ids),H.margin');
  slope=min(cdc_dynamic_slope(xa,Q),[],1);minC(ids)=min(minC(ids),slope);
  prepeak(ids)=prepeak(ids)&(slope>0);hit=ids(~(slope>0));firstSlope(hit)=min(firstSlope(hit),t);
  hit=ids(bitand(bits,uint16(2))~=0);firstAy(hit)=min(firstAy(hit),t);
  hit=ids(bitand(bits,uint16(1))~=0);firstYaw(hit)=min(firstYaw(hit),t);
  peakR(ids)=max(peakR(ids),abs(xa(2,:)));peakAy(ids)=max(peakAy(ids),abs(O.ay));
  peakLTR(ids)=max(peakLTR(ids),max(abs([O.LTR;O.LTRf;O.LTRr]),[],1));minFz(ids)=min(minFz(ids),min(O.Fz_raw,[],1));observed(ids)=t;
  if usable
   err(ids)=max(abs(xa(1:29,:)-T.x(1:29))./c.state_tolerance(:),[],1);
   derr(ids)=max(abs(k1(1:29,:))./(c.state_tolerance(:)/c.derivative_time_scale),[],1);
   hit=ids(err(ids)>1|derr(ids)>1);lastNot(hit)=t;
  end
  bad=bitand(bits,uint16(252))~=0;hit=ids(bad);active(hit)=false;firstHard(hit)=min(firstHard(hit),t);
  if c.trace_stride>0&&(mod(k,c.trace_stride)==0||k==steps(end)||any(bad))
   trace=[trace;[ids',t*ones(numel(ids),1),xa([1 2 4 11],:)',O.ay',O.LTR',minFz(ids)',H.margin,slope',err(ids)',derr(ids)']]; %#ok<AGROW>
  end
  good=~bad;ids=ids(good);xa=xa(:,good);k1=k1(:,good);
 end
 if ih<=nh&&k==steps(ih)
  records(:,ih,:)=reshape(snapshot(),n,1,nf);ih=ih+1;
 end
 if k==steps(end),break;end
 if isempty(ids)
  while ih<=nh,records(:,ih,:)=reshape(snapshot(),n,1,nf);ih=ih+1;end
  break;
 end
 if feedback
  k2=cdc_fullcar_model('rhs_current',xa+0.5*dt*k1,Q,I(:,ids));
  k3=cdc_fullcar_model('rhs_current',xa+0.5*dt*k2,Q,I(:,ids));
  k4=cdc_fullcar_model('rhs_current',xa+dt*k3,Q,I(:,ids));
 else
  k2=cdc_fullcar_model('rhs',xa+0.5*dt*k1,Q);
  k3=cdc_fullcar_model('rhs',xa+0.5*dt*k2,Q);
  k4=cdc_fullcar_model('rhs',xa+dt*k3,Q);
 end
 X(:,ids)=xa+dt*(k1+2*k2+2*k3+k4)/6;
end
names=cdc_research_fields();out=struct();
for j=1:nf,out.(names{j})=records(:,:,j);end
out.final_state=X';out.trace=trace;
 function row=snapshot()
  ms=active;sa=ms&bitand(reason,uint16(2))==0;ya=bitand(reason,uint16(1))==0;
  close=usable&(t-lastNot>=c.terminal_window-1e-10)&err<=1&derr<=1;
  mr=ms&close;re=sa&close;hp=re&kept;
  row=[double(sa);double(ms);double(ya);double(mr);double(re);double(re&ya);double(hp);double(hp&ya);double(hp&prepeak);err;derr;firstH;outside;minMargin;firstAy;firstYaw;firstHard;peakR;peakAy;peakLTR;minFz;minC;firstSlope;double(reason);observed;double(kept)]';
 end
end
