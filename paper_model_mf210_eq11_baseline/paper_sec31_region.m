function region=paper_sec31_region(stable_mask,G)
%PAPER_SEC31_REGION Explicit discrete completion of paper Sec3.1 Steps1-4.
% No volume objective, equilibrium anchor, rank erosion, cross-angle nesting,
% coordinate correction, roll cap, or fitted aspect ratio is used.
% The publication omits executable branch/eight-corner tie rules. These
% numerical choices are documented in README.md and retained as diagnostics.
% Invalid extreme-point candidates are saved, never silently replaced.
  m=logical(stable_mask);names={'vc','r','phi'};sz=zeros(1,3);
  for k=1:3
    G.(names{k})=double(G.(names{k})(:));g=G.(names{k});sz(k)=numel(g);
    assert(all(isfinite(g))&&all(diff(g)>0),'Finite ascending grid required');
  end
  assert(isequal(size(m),sz),'Grid/mask size mismatch');
  lower=zeros(sz(1)*sz(2),3);upper=lower;n=0;censored=0;disconnected=0;
  for v=1:sz(1)
    for r=1:sz(2)
      ps=find(reshape(m(v,r,:),[],1));if isempty(ps),continue;end
      n=n+1;lower(n,:)=[v r ps(1)]-1;upper(n,:)=[v r ps(end)]-1;
      censored=censored+(ps(1)==1||ps(end)==sz(3));
      disconnected=disconnected+any(diff(ps)>1);
    end
  end
  lower=lower(1:n,:);upper=upper(1:n,:);limit=unique([lower;upper],'rows');
  outer=zeros(0,3);inner=zeros(0,3);rows=cell(0,1);
  counts=struct('one_band_coincident',0,'two_bands_central_pair',0, ...
    'multiple_bands_ambiguous_excluded',0,'central_interval_contains_unstable_excluded',0);
  rps=unique(limit(:,2:3),'rows');
  for ii=1:size(rps,1)
    r=rps(ii,1);p=rps(ii,2);vs=sort(limit(limit(:,2)==r&limit(:,3)==p,1));
    cuts=[0;find(diff(vs)>1);numel(vs)];nb=numel(cuts)-1;
    outer=[outer;vs(1) r p;vs(end) r p]; %#ok<AGROW>
    row=struct('r_index',r,'phi_index',p,'band_count',nb,'status','','inner_vc_indices',zeros(1,0));
    if nb==1
      pair=[vs(1) vs(end)];counts.one_band_coincident=counts.one_band_coincident+1;
      row.status='one_band_inner_outer_coincident_at_grid_resolution';
    elseif nb==2
      pair=[vs(cuts(2)) vs(cuts(2)+1)];
      if ~all(m(pair(1)+1:pair(2)+1,r+1,p+1))
        pair=[];counts.central_interval_contains_unstable_excluded=counts.central_interval_contains_unstable_excluded+1;
        row.status='central_gap_not_all_stable_excluded';
      else
        counts.two_bands_central_pair=counts.two_bands_central_pair+1;
        row.status='facing_endpoints_of_two_branches';
      end
    else
      pair=[];counts.multiple_bands_ambiguous_excluded=counts.multiple_bands_ambiguous_excluded+1;
      row.status='multiple_boundary_branches_ambiguous_excluded';
    end
    row.inner_vc_indices=pair;rows{end+1,1}=row; %#ok<AGROW>
    for v=pair,inner(end+1,:)=[v r p];end %#ok<AGROW>
  end
  outer=unique(outer,'rows');inner=unique(inner,'rows');
  [v,r,p]=ind2sub(sz,find(~m));bad=coords_idx([v r p]-1,G);
  a=eight_from_sections(inner,G,bad);b=eight_from_sections(outer,G,bad);
  region=struct('method','Paper Sec3.1 extreme-point sequence with declared discrete branch/corner completion', ...
    'author_exact_algorithm_claimed',false,'max_volume_objective_used',false,'equilibrium_anchor_used',false, ...
    'excluded_node_definition','Every node not classified stable; may include physically unstable and numerically unresolved nodes', ...
    'nesting_constraints_used',false,'roll_scaling_or_caps_applied',false, ...
    'limit_roll_points',coords_idx(limit,G),'lower_roll_points',coords_idx(lower,G), ...
    'upper_roll_points',coords_idx(upper,G),'outer_limit_points',coords_idx(outer,G), ...
    'inner_limit_points',coords_idx(inner,G),'lower_roll_indices_zero_based',lower, ...
    'upper_roll_indices_zero_based',upper,'limit_roll_indices_zero_based',limit, ...
    'outer_limit_indices_zero_based',outer,'inner_limit_indices_zero_based',inner, ...
    'slice_diagnostics',{rows},'branch_counts',counts,'stable_vc_yaw_slices',n, ...
    'roll_domain_censored_slices',censored,'disconnected_roll_slices',disconnected, ...
    'physical_roll_boundary_resolved',n>0&&censored==0,'inner_candidate',a,'outer_candidate',b, ...
    'is_valid_inner',a.valid,'continuous_stability_certificate',false);
end

function p=coords_idx(idx,G)
  p=zeros(size(idx,1),3);names={'vc','r','phi'};
  for k=1:3,p(:,k)=G.(names{k})(idx(:,k)+1);end
end

function out=eight_from_sections(idx,G,bad)
  idx=unique(idx,'rows');sections=cell(0,1);
  for r=unique(idx(:,2)).'
    a=idx(idx(:,2)==r,:);p=[G.vc(a(:,1)+1),G.phi(a(:,3)+1)];
    [four,rule]=section_four(p);
    if ~isempty(four),sections{end+1,1}=struct('r',r,'four',four,'rule',rule);end %#ok<AGROW>
  end
  out=struct('valid',false,'vertices',zeros(0,3),'faces',zeros(0,4),'volume',NaN, ...
    'unavailable_reason','Fewer than two yaw sections admit four distinct convex extrema');
  if numel(sections)<2,return;end
  lo=sections{1};hi=sections{end};v=[lo.four(:,1),repmat(G.r(lo.r+1),4,1),lo.four(:,2); ...
                                    hi.four(:,1),repmat(G.r(hi.r+1),4,1),hi.four(:,2)];
  out=hull_audit(v,bad);out.yaw_section_indices_zero_based=[lo.r hi.r];
  out.yaw_section_selection='First and last yaw sections admitting four nondegenerate extrema; no inward search';
  out.cap_selection_rules={lo.rule,hi.rule};
  if ~out.valid,out.unavailable_reason='Extreme-point candidate fails eight-vertex/six-quad/zero-unstable audit';end
end

function [four,rule]=section_four(p)
  p=unique(p,'rows');four=zeros(0,2);rule='fewer_than_four_nondegenerate_points';
  if size(p,1)<4||rank(bsxfun(@minus,p,mean(p,1)),1e-11)<2,return;end
  [h,ok]=canonical_polygon(p);if ~ok,rule='degenerate_section';return;end
  if size(h,1)==4,four=h;rule='exact_four_vertex_section_hull';return;end
  selected=zeros(4,2);axes=[1 2 1 2];signs=[-1 1 1 -1];
  for ii=1:4
    a=axes(ii);s=signs(ii);value=max(p(:,a)*s);hits=p(abs(p(:,a)*s-value)<=1e-12,:);
    other=3-a;mid=(min(p(:,other))+max(p(:,other)))/2;
    order=sortrows([abs(hits(:,other)-mid),hits],[1 2 3]);selected(ii,:)=order(1,2:3);
  end
  selected=unique(selected,'rows');
  if size(selected,1)~=4,rule='cardinal_extrema_not_four_distinct_points';return;end
  [h,ok]=canonical_polygon(selected);
  if ~ok,rule='cardinal_extrema_degenerate';return;end
  if size(h,1)~=4,rule='cardinal_extrema_not_convex_four_vertices';return;end
  four=h;rule='four_cardinal_vc_phi_extrema';
end

function [q,ok]=canonical_polygon(p)
  q=zeros(0,2);ok=false;
  try,k=convhull(p(:,1),p(:,2));catch,return;end
  q=p(k(1:end-1),:);j=[2:size(q,1),1];area=sum(q(:,1).*q(j,2)-q(:,2).*q(j,1));
  if area<0,q=flipud(q);end
  [~,order]=sortrows(q,[1 2]);start=order(1);q=q([start:end,1:start-1],:);ok=true;
end

function out=hull_audit(v,bad)
  out=struct('valid',false,'vertices',v,'faces',zeros(0,4),'volume',NaN, ...
    'extreme_vertices',0,'planar_face_count',0,'unstable_nodes_inside_inclusive',-1, ...
    'excluded_not_stable_nodes_inside_inclusive',-1, ...
    'excluded_node_definition','Every node not classified stable; may include physically unstable and numerically unresolved nodes');
  if size(v,1)~=8||size(unique(v,'rows'),1)~=8,return;end
  try,[tri,vol]=convhulln(v);catch,return;end
  center=mean(v,1);planes=zeros(0,4);
  for ii=1:size(tri,1)
    p=v(tri(ii,:),:);n=cross(p(2,:)-p(1,:),p(3,:)-p(1,:));normn=norm(n);if normn<1e-14,continue;end
    n=n/normn;if dot(n,center-p(1,:))>0,n=-n;end
    eq=[n -dot(n,p(1,:))];
    if isempty(planes)||~any(max(abs(bsxfun(@minus,planes,eq)),[],2)<1e-9),planes(end+1,:)=eq;end %#ok<AGROW>
  end
  count=0;
  for start=1:20000:size(bad,1)
    pts=bad(start:min(start+19999,size(bad,1)),:);
    count=count+nnz(all(bsxfun(@plus,pts*planes(:,1:3).',planes(:,4).')<=1e-10,2));
  end
  faces=zeros(0,4);
  for ii=1:size(planes,1)
    eq=planes(ii,:);ids=find(abs(v*eq(1:3).'+eq(4))<1e-9);
    if numel(ids)~=4,continue;end
    pts=v(ids,:);c=mean(pts,1);u=pts(1,:)-c;u=u/norm(u);w=cross(eq(1:3),u);
    z=bsxfun(@minus,pts,c);ang=atan2(z*w.',z*u.');[~,ord]=sort(ang);cycle=ids(ord).';
    [~,first]=min(cycle);cycle=cycle([first:end,1:first-1]);
    if cycle(2)>cycle(end),cycle=[cycle(1),cycle(end:-1:2)];end
    faces(end+1,:)=cycle; %#ok<AGROW>
  end
  faces=sortrows(faces);extreme=numel(unique(tri(:)));
  out.volume=vol;out.extreme_vertices=extreme;out.planar_face_count=size(planes,1);
  out.hull_triangles_1based=tri;out.unstable_nodes_inside_inclusive=count;
  out.excluded_not_stable_nodes_inside_inclusive=count;
  if size(faces,1)==6,out.faces=faces;end
  out.valid=extreme==8&&size(planes,1)==6&&size(faces,1)==6&&count==0&&vol>1e-14;
end
