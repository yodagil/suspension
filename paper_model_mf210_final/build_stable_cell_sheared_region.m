function result=build_stable_cell_sheared_region(stable_mask,G,equilibrium_point)
%BUILD_STABLE_CELL_SHEARED_REGION Independent conservative eight-vertex region.
% Uniform ascending grids G.vc,G.r,G.phi, with mask order [vc,r,phi].
% A six-connected stable component containing the nearest equilibrium node
% provides vc(r) and phi(r) slopes by equal-yaw-row least squares. For every
% yaw interval containing the actual equilibrium, endpoint grid shifts are
% independently selected from {0,floor(slope*interval),ceil(slope*interval)}.
% Maximize volume, breaking ties by the zero-based tuple
% [vc_lo,vc_hi,r_lo,r_hi,phi_lo,phi_hi,vc_shift,phi_shift].
%
% Every accepted shape avoids the conservative inverse-sheared bounding-box
% projections of physical unit cells that do not have eight stable corners.
% It also excludes all raw bad nodes, including boundaries. This is a
% sampled finite-horizon construction, not a continuous-stability proof.
% No other steering angle, target image, nesting condition, roll cap,
% coordinate correction, angle factor or prescribed aspect ratio is used.
  mask=logical(stable_mask); coordinates={G.vc(:),G.r(:),G.phi(:)};
  sz=cellfun(@numel,coordinates);nv=sz(1);nr=sz(2);np=sz(3);
  assert(all(sz>=2)&&isequal(size(mask),sz),'Mask/grid size mismatch');
  origin=zeros(1,3);step=zeros(1,3);
  for k=1:3
    g=coordinates{k};origin(k)=g(1);step(k)=g(2)-g(1);
    assert(all(isfinite(g))&&step(k)>0&&all(abs(diff(g)-step(k))<1e-12), ...
      'Uniform finite ascending grids are required');
  end
  anchor=double(equilibrium_point(:).');
  assert(numel(anchor)==3&&all(isfinite(anchor)),'Finite [vc,r,phi] anchor required');
  ai=(anchor-origin)./step;
  result=empty_result(anchor);
  if any(ai< -1e-10)||any(ai>sz-1+1e-10)
    result.unavailable_reason='Equilibrium is outside the grid';return;
  end
  nearest=zeros(1,3);
  for k=1:3,[~,nearest(k)]=min(abs(coordinates{k}-anchor(k)));end
  if ~mask(nearest(1),nearest(2),nearest(3))
    result.unavailable_reason='Nearest equilibrium grid node is unstable';return;
  end
  % Base-MATLAB six-connected flood fill; no Image Processing Toolbox.
  component=false(sz);queue=zeros(nnz(mask),1);head=1;tail=1;
  queue(1)=sub2ind(sz,nearest(1),nearest(2),nearest(3));component(queue(1))=true;
  while head<=tail
    idx=queue(head);head=head+1;[v,r,p]=ind2sub(sz,idx);
    neighbors=[v-1 r p;v+1 r p;v r-1 p;v r+1 p;v r p-1;v r p+1];
    inside=all(neighbors>=1,2)&neighbors(:,1)<=nv&neighbors(:,2)<=nr&neighbors(:,3)<=np;
    neighbors=neighbors(inside,:);
    ids=sub2ind(sz,neighbors(:,1),neighbors(:,2),neighbors(:,3));
    ids=ids(mask(ids)&~component(ids));
    for ii=1:numel(ids)
      if ~component(ids(ii)),tail=tail+1;queue(tail)=ids(ii);component(ids(ii))=true;end
    end
  end
  rows=zeros(nr,3);nrows=0;
  for r=1:nr
    [v,p]=find(reshape(component(:,r,:),nv,np));
    if ~isempty(v),nrows=nrows+1;rows(nrows,:)=[r-1,mean(v-1),mean(p-1)];end
  end
  rows=rows(1:nrows,:);slopes=[0 0];
  if nrows>=2
    centered=rows(:,1)-mean(rows(:,1));den=sum(centered.^2);
    slopes=[sum(centered.*(rows(:,2)-mean(rows(:,2)))), ...
            sum(centered.*(rows(:,3)-mean(rows(:,3))))]/den;
  end
  % Prefix sums of unsafe PHYSICAL unit cells, once per physical yaw slab.
  safe=mask(1:end-1,1:end-1,1:end-1)&mask(2:end,1:end-1,1:end-1)& ...
       mask(1:end-1,2:end,1:end-1)&mask(2:end,2:end,1:end-1)& ...
       mask(1:end-1,1:end-1,2:end)&mask(2:end,1:end-1,2:end)& ...
       mask(1:end-1,2:end,2:end)&mask(2:end,2:end,2:end);
  prefix=zeros(nv,np,nr-1);
  for r=1:nr-1
    plane=double(~reshape(safe(:,r,:),nv-1,np-1));
    prefix(2:end,2:end,r)=cumsum(cumsum(plane,1),2);
  end
  best=-1;best_bounds=[];ties=0;minroll=Inf;maxroll=-Inf;configs=0;rectangles=0;
  for r0=0:nr-2
    if r0>ai(2)+1e-10,continue;end
    for r1=r0+1:nr-1
      if r1<ai(2)-1e-10,continue;end
      L=r1-r0;dvs=unique([0 floor(slopes(1)*L) ceil(slopes(1)*L)]);
      dps=unique([0 floor(slopes(2)*L) ceil(slopes(2)*L)]);
      for sv=dvs
        for sp=dps
          vl=max(0,-sv);vh=min(nv-1,nv-1-sv);pl=max(0,-sp);ph=min(np-1,np-1-sp);
          V=vh-vl;P=ph-pl;if V<=0||P<=0,continue;end
          qav=ai(1)-sv*(ai(2)-r0)/L;qap=ai(3)-sp*(ai(2)-r0)/L;
          if qav<vl-1e-10||qav>vh+1e-10||qap<pl-1e-10||qap>ph+1e-10,continue;end
          configs=configs+1;good=true(V,P);vi=(vl:vh-1).';pi=(pl:ph-1);
          for r=r0:r1-1
            % Dual of C++ inverse-bbox rasterization: test each q cell's
            % physical vc/phi bbox against unsafe physical unit cells.
            shiftsv=sv*[r-r0,r+1-r0];shiftsp=sp*[r-r0,r+1-r0];
            vlo=floor((vi*L+min(shiftsv))/L);vhi=ceil(((vi+1)*L+max(shiftsv))/L)-1;
            plo=floor((pi*L+min(shiftsp))/L);phi=ceil(((pi+1)*L+max(shiftsp))/L)-1;
            S=prefix(:,:,r+1);
            i11=bsxfun(@plus,vhi+2,nv*(phi+1));i01=bsxfun(@plus,vlo+1,nv*(phi+1));
            i10=bsxfun(@plus,vhi+2,nv*plo);i00=bsxfun(@plus,vlo+1,nv*plo);
            good=good&(S(i11)-S(i01)-S(i10)+S(i00)==0);
          end
          % Unsafe-cell avoidance implies bad-node exclusion: every raw
          % boundary node belongs to the closure of some intersected cell,
          % whose eight corners must all be stable. Test anchor-adjacent
          % q cells before scanning maximal rectangles.
          avc=incident_cells(qav,vl,vh);apc=incident_cells(qap,pl,ph);
          if isempty(avc)||isempty(apc)||~any(any(good(avc-vl+1,apc-pl+1))),continue;end
          activev=find(any(good,2));activep=find(any(good,1));
          if isempty(activev)||isempty(activep),continue;end
          va=activev(1);vb=activev(end);pa=activep(1);pb=activep(end);
          cropped=good(va:vb,pa:pb);W=size(cropped,2);heights=zeros(1,W);
          for row=1:size(cropped,1)
            heights(~cropped(row,:))=0;heights(cropped(row,:))=heights(cropped(row,:))+1;
            starts=zeros(1,W);hs=zeros(1,W);top=0;
            for rend=1:W+1
              if rend<=W,height=heights(rend);else,height=0;end
              start=rend;
              while top>0&&hs(top)>height
                pstart=starts(top);h=hs(top);top=top-1;start=pstart;
                v0=vl+va+row-h-1;v1=vl+va+row-1;
                p0=pl+pa+pstart-2;p1=pl+pa+rend-2;
                if ~(v0<=qav+1e-10&&v1>=qav-1e-10&&p0<=qap+1e-10&&p1>=qap-1e-10),continue;end
                rectangles=rectangles+1;volume=(v1-v0)*(p1-p0)*L;
                b=[v0 v1 r0 r1 p0 p1 sv sp];roll=p1-p0+abs(sp);
                if volume>best
                  best=volume;best_bounds=b;ties=1;minroll=roll;maxroll=roll;
                elseif volume==best
                  ties=ties+1;minroll=min(minroll,roll);maxroll=max(maxroll,roll);
                  if lex_less(b,best_bounds),best_bounds=b;end
                end
              end
              if height>0&&(top==0||hs(top)<height)
                top=top+1;starts(top)=start;hs(top)=height;
              end
            end
          end
        end
      end
    end
  end
  result.fitted_grid_slopes=slopes;result.six_connected_component_nodes=nnz(component);
  result.configurations=configs;result.rectangles=rectangles;
  if isempty(best_bounds),result.unavailable_reason='No positive-volume candidate contains equilibrium';return;end
  b=best_bounds;iv=zeros(8,3);index=0;
  for v=b(1:2)
    for t=0:1
      r=b(3+t);
      for p=b(5:6)
        index=index+1;iv(index,:)=[v+b(7)*t,r,p+b(8)*t];
      end
    end
  end
  vertices=zeros(8,3);
  for k=1:3,vertices(:,k)=coordinates{k}(iv(:,k)+1);end
  lower=min(vertices,[],1);upper=max(vertices,[],1);
  result.valid=true;result.is_valid_inner=true;result.vertices=vertices;result.inner_vertices=vertices;
  result.faces=[1 2 4 3;5 7 8 6;1 5 6 2;3 4 8 7;1 3 7 5;2 6 8 4];
  result.volume=best*prod(step);result.inner_volume=result.volume;result.volume_grid_units=best;
  result.grid_bound_indices_zero_based=b;result.optimizer_bounds_zero_based=b;
  result.bounds=[lower(:),upper(:)];result.lower_bounds=lower;result.upper_bounds=upper;
  result.coordinate_spans=upper-lower;result.coordinate_centers=(upper+lower)/2;
  result.roll_bounds=[lower(3),upper(3)];result.roll_span=upper(3)-lower(3);
  result.roll_center=(lower(3)+upper(3))/2;result.maximum_volume_tie_count=ties;
  result.tie_roll_span_grid_min=minroll;result.tie_roll_span_grid_max=maxroll;
  result.maximum_volume_ties_roll_span_range=[minroll maxroll]*step(3);
  result.contains_equilibrium=true;result.unstable_nodes_inside_inclusive=0;
  result.all_physical_cells_intersected_have_eight_stable_corners=true;
  result.global_maximum_within_specified_conservative_candidate_family=true;
end

function c=incident_cells(x,lo,hi)
  if abs(x-round(x))<=1e-10,c=[round(x)-1 round(x)];else,c=floor(x);end
  c=c(c>=lo&c<hi);
end
function yes=lex_less(a,b)
  i=find(a~=b,1);yes=~isempty(i)&&a(i)<b(i);
end
function result=empty_result(anchor)
  result=struct('valid',false,'is_valid_inner',false,'volume',NaN,'inner_volume',NaN, ...
    'vertices',zeros(0,3),'inner_vertices',zeros(0,3),'faces',zeros(0,4), ...
    'bounds',NaN(3,2),'equilibrium_anchor_vc_r_phi',anchor, ...
    'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
    'nesting_constraints_applied',false,'roll_scaling_or_caps_applied',false, ...
    'coordinate_correction_applied',false,'continuous_stability_certificate',false, ...
    'method','Maximum-volume conservative stable-cell sheared grid box', ...
    'orientation_rule','Six-connected component row means; OLS slopes; endpoint shifts zero/floor/ceil', ...
    'tie_rule','Lexicographic [vc0,vc1,r0,r1,phi0,phi1,vc_shift,phi_shift]');
end
