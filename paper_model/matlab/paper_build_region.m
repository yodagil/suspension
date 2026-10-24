function reg=paper_build_region(cfg,stable_mask)
%PAPER_BUILD_REGION Boundary-derived eight-actual-vertex inner reconstruction.
% Implements the published order: roll limits -> lateral-speed limit sets ->
% eight extreme vertices -> inner region with no sampled unstable points.
% Inward vc ranks, normalized octants and support ranks are explicit discrete
% reconstruction assumptions; the paper does not publish these numeric rules.
% No PCA box, off-grid vertex, or steering-dependent shrink factor is used.
G=cfg.grid;
[nv,nr,np]=size(stable_mask);
assert(isequal([nv nr np],[numel(G.vc),numel(G.r),numel(G.phi)]),'Grid/mask shape mismatch.');
[VC,RR,PHI]=ndgrid(G.vc,G.r,G.phi);
P=[VC(:) RR(:) PHI(:)]; stable=logical(stable_mask(:)); S=P(stable,:);
scale=[max(G.vc)-min(G.vc),max(G.r)-min(G.r),max(G.phi)-min(G.phi)];
assert(all(scale>0),'A three-dimensional grid is required.');
PN=bsxfun(@rdivide,P,scale); UN=PN(~stable,:);
tol=1e-10;
reg=struct('vertices',zeros(0,3),'faces',zeros(0,3),'volume',NaN, ...
    'valid',false,'grid_unstable_inside',NaN,'roll_limit_points',zeros(0,3), ...
    'inner_limit_points',zeros(0,3),'outer_limit_points',zeros(0,3), ...
    'method','roll-limit inward-vc ranks and eight actual stable support vertices', ...
    'status','no_valid_eight_vertex_inner_region','sampled_grid_only',true, ...
    'inner_depth',NaN,'support_rank',NaN,'samples_inside',0, ...
    'vertex_grid_indices',zeros(0,1),'candidates_tested',0, ...
    'outer_vertices',zeros(0,3),'outer_volume',NaN,'outer_grid_unstable_inside',NaN, ...
    'grid_domain_volume',prod(scale),'raw_stable_hull_volume',0, ...
    'axis_spans',zeros(1,3),'boundary_contact_counts',zeros(3,2), ...
    'boundary_contact',false,'center',nan(1,3));
reg.boundary_contact_counts=[nnz(stable_mask(1,:,:)),nnz(stable_mask(end,:,:)); ...
    nnz(stable_mask(:,1,:)),nnz(stable_mask(:,end,:)); ...
    nnz(stable_mask(:,:,1)),nnz(stable_mask(:,:,end))];
reg.boundary_contact=any(reg.boundary_contact_counts(:)>0);
if isempty(S), reg.status='no_stable_grid_samples'; return; end
reg.axis_spans=max(S,[],1)-min(S,[],1);
reg.center=mean(S,1);
SN=bsxfun(@rdivide,S,scale);
if size(S,1)>=4 && rank(bsxfun(@minus,SN,mean(SN,1)))==3
    try
        [~,raw_volume]=convhulln(SN);
        reg.raw_stable_hull_volume=raw_volume*prod(scale);
    catch
        reg.raw_stable_hull_volume=NaN;
    end
end
if size(S,1)<8 || rank(bsxfun(@minus,SN,mean(SN,1)))<3
    reg.status='insufficient_three_dimensional_stable_samples'; return;
end
% Paper step 1: actual minimum/maximum stable roll angles at fixed (vc,r).
roll=false(size(stable_mask));
for iv=1:nv
    for ir=1:nr
        ip=find(squeeze(stable_mask(iv,ir,:)));
        if ~isempty(ip)
            roll(iv,ir,ip(1))=true;
            roll(iv,ir,ip(end))=true;
        end
    end
end
reg.roll_limit_points=P(roll(:),:);
% Preset assumption used identically for every steering condition.
max_depth=ceil(nv/2); max_support_rank=12;
if isfield(cfg,'region_max_support_rank')
    max_support_rank=cfg.region_max_support_rank;
end
signs=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1];
best=-Inf;
for depth=1:max_depth
    % Paper step 2: inward-ranked vc endpoints within the roll-limit set.
    % depth=1 is the outer set and can also be inner if already entirely safe.
    ids=zeros(2*nr*np,1); used=0;
    for ir=1:nr
        for ip=1:np
            iv=find(squeeze(roll(:,ir,ip)));
            if numel(iv)<2*depth-1, continue; end
            pair=[iv(depth),iv(numel(iv)-depth+1)];
            for j=1:2
                used=used+1;
                ids(used)=sub2ind([nv nr np],pair(j),ir,ip);
            end
        end
    end
    ids=unique(ids(1:used));
    if depth==1, reg.outer_limit_points=P(ids,:); end
    if numel(ids)<8, continue; end
    Q=PN(ids,:); center=mean(Q,1); Qc=bsxfun(@minus,Q,center);
    if rank(Qc)<3, continue; end
    orders=cell(8,1);
    octant_ok=true;
    for k=1:8
        in_octant=all(bsxfun(@times,Qc,signs(k,:))>=-tol,2);
        ix=find(in_octant);
        if isempty(ix), octant_ok=false; break; end
        score=Qc(ix,:)*signs(k,:).';
        [~,ord]=sortrows([-score ids(ix)],[1 2]);
        orders{k}=ix(ord);
    end
    if ~octant_ok, continue; end
    for support_rank=1:max_support_rank
        chosen=zeros(8,1); good=true;
        for k=1:8
            pool=orders{k}; pool=pool(~ismember(pool,chosen(1:k-1)));
            if numel(pool)<support_rank, good=false; break; end
            chosen(k)=pool(support_rank);
        end
        if ~good, continue; end
        vertex_ids=ids(chosen); VN=PN(vertex_ids,:);
        if numel(unique(vertex_ids))~=8 || rank(bsxfun(@minus,VN,mean(VN,1)))<3
            continue;
        end
        try
            [K,normalized_volume]=convhulln(VN);
        catch
            continue;
        end
        % Every one of the eight selected samples must be a hull vertex.
        if numel(unique(K(:)))~=8 || ~(isfinite(normalized_volume)&&normalized_volume>0)
            continue;
        end
        reg.candidates_tested=reg.candidates_tested+1;
        volume=normalized_volume*prod(scale);
        assert(volume<=reg.grid_domain_volume*(1+1e-9),'Region extrapolates outside tested grid.');
        bad=nnz(inside_hull(UN,VN,K,tol));
        if depth==1 && support_rank==1
            reg.outer_vertices=P(vertex_ids,:);
            reg.outer_volume=volume;
            reg.outer_grid_unstable_inside=bad;
        end
        if bad==0 && volume>best
            best=volume;
            reg.vertices=P(vertex_ids,:); reg.faces=K; reg.volume=volume;
            reg.grid_unstable_inside=0; reg.valid=true;
            reg.inner_limit_points=P(ids,:); reg.inner_depth=depth;
            reg.support_rank=support_rank; reg.vertex_grid_indices=vertex_ids;
            reg.samples_inside=nnz(inside_hull(PN,VN,K,tol));
            reg.center=mean(reg.vertices,1);
            reg.status='valid_sampled_inner_polytope';
        end
    end
end
if reg.valid
    assert(all(stable(reg.vertex_grid_indices)),'A selected vertex was not a stable sample.');
    assert(reg.grid_unstable_inside==0,'Unsafe sampled inner polytope.');
end
end

function inside=inside_hull(P,V,K,tol)
% Normalize coordinates before calling; use actual hull facets and inclusive
% boundaries. Bounding-box rejection is exact and only accelerates the test.
inside=false(size(P,1),1);
if isempty(P), return; end
inside=all(bsxfun(@ge,P,min(V,[],1)-tol)&bsxfun(@le,P,max(V,[],1)+tol),2);
c=mean(V,1);
for k=1:size(K,1)
    ix=find(inside);
    if isempty(ix), break; end
    tri=V(K(k,:),:);
    n=cross(tri(2,:)-tri(1,:),tri(3,:)-tri(1,:));
    length_n=norm(n);
    if length_n<=eps, continue; end
    n=n/length_n;
    if dot(n,c-tri(1,:))>0, n=-n; end
    d=bsxfun(@minus,P(ix,:),tri(1,:))*n.';
    inside(ix)=d<=tol;
end
end
