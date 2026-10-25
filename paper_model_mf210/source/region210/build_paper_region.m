function region = build_paper_region(G,stable_mask,stable_points,unstable_points,cfg)

    % Robust reconstruction of the paper sequence:
    % Stable -> limit-roll -> lateral-speed layers -> eight vertices
    % -> inner/outer hexahedra.
    %
    % Difference from the previous implementation:
    % The old code selected only one "inner" layer.  In the last run that
    % hexahedron contained 7 sampled unstable points, so the script stopped.
    % Here the inner layer is moved inward discretely (depth=2,3,4,and onward) and
    % the largest candidate containing ZERO sampled unstable points is kept.

    region = empty_region();

    if isempty(stable_points) || size(stable_points,1)<8
        return;
    end

    if rank(stable_points-mean(stable_points,1))<3
        return;
    end

    % ---------------------------------------------------------------------
    % STEP 1: LIMIT-ROLL STABLE POINTS
    % For each fixed (vc,r), take the minimum/maximum stable phi.
    % ---------------------------------------------------------------------
    limit_roll = zeros(0,3);

    for iv=1:numel(G.vc)
        for ir=1:numel(G.r)

            idxPhi = find(squeeze(stable_mask(iv,ir,:)));

            if isempty(idxPhi)
                continue;
            end

            ipLo = idxPhi(1);
            ipHi = idxPhi(end);

            limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipLo)]; %#ok<AGROW>

            if ipHi~=ipLo
                limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipHi)]; %#ok<AGROW>
            end
        end
    end

    limit_roll = unique(limit_roll,'rows','stable');
    region.limit_roll_points = limit_roll;

    if size(limit_roll,1)<cfg.min_limit_points
        return;
    end

    % Keep this diagnostic center for compatibility with older MAT files.
    region.vc_split_center = median(stable_points(:,1));

    % ---------------------------------------------------------------------
    % STEP 2A: OUTER LATERAL-SPEED LIMIT
    % depth=1 means the outermost stable vc point on both sides.
    % ---------------------------------------------------------------------
    outer_limit = lateral_layer_from_limit_roll(limit_roll,G,1);
    usedFallback = false;

    if size(outer_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

        outer_limit = lateral_layer_from_stable_mask(G,stable_mask,1);
        usedFallback = true;
    end

    outer_limit = unique(outer_limit,'rows','stable');
    region.outer_limit_points = outer_limit;
    region.used_stable_mask_fallback = usedFallback;

    if size(outer_limit,1)<cfg.min_limit_points
        return;
    end

    Vouter = select_ordered_eight_vertices(outer_limit,G,stable_points);

    if size(Vouter,1)~=8
        return;
    end

    [outerVol,okOuter] = convex_volume(Vouter);

    if ~okOuter
        return;
    end

    insideOuter = points_inside_convex_hull( unstable_points,Vouter,cfg.hull_inside_tol);

    region.outer_vertices = Vouter;
    region.outer_volume = outerVol;
    region.unstable_inside_outer = nnz(insideOuter);

    % ---------------------------------------------------------------------
    % STEP 2B / STEP 3: INNER LATERAL-SPEED LAYER SEARCH
    %
    % depth=1 : outermost layer
    % depth=2 : one stable layer inward
    % depth=3 : two stable layers inward
    % and onward
    %
    % Among all candidate hexahedra containing zero sampled unstable points,
    % keep the LARGEST volume.  Thus we do not shrink more than necessary.
    % ---------------------------------------------------------------------
    maxDepth = max(1,ceil(numel(G.vc)/2));

    bestVol = -Inf;
    best = struct( 'limit',zeros(0,3), 'V',zeros(0,3), 'bad',inf, 'depth',NaN, 'fallback',false);

    for depth = cfg.inner_depth_start:maxDepth

        inner_limit = lateral_layer_from_limit_roll(limit_roll,G,depth);
        thisFallback = false;

        if size(inner_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

            inner_limit = lateral_layer_from_stable_mask( G,stable_mask,depth);
            thisFallback = true;
        end

        inner_limit = unique(inner_limit,'rows','stable');

        if size(inner_limit,1)<cfg.min_limit_points
            continue;
        end

        Vin = select_ordered_eight_vertices( inner_limit,G,stable_points);

        if size(Vin,1)~=8
            continue;
        end

        [v,ok] = convex_volume(Vin);

        if ~ok || ~(isfinite(v) && v>0)
            continue;
        end

        bad = nnz(points_inside_convex_hull( unstable_points,Vin,cfg.hull_inside_tol));

        fprintf('  Inner search depth=%2d | points=%4d | volume=%10.6g | bad=%d\n', depth,size(inner_limit,1),v,bad);

        if bad==0 && v>bestVol
            bestVol = v;
            best.limit = inner_limit;
            best.V = Vin;
            best.bad = bad;
            best.depth = depth;
            best.fallback = thisFallback;
        end
    end

    % If the outer layer itself is already safe, allow it as a valid region.
    if isempty(best.V) && region.unstable_inside_outer==0
        bestVol = outerVol;
        best.limit = outer_limit;
        best.V = Vouter;
        best.bad = 0;
        best.depth = 1;
        best.fallback = usedFallback;
    end

    % ---------------------------------------------------------------------
    % LAST-RESORT GEOMETRIC FALLBACK
    % Search directly among saved stable points with progressively inward
    % support points.  No vehicle simulation is rerun.
    % ---------------------------------------------------------------------
    if isempty(best.V)

        [Vs,bads,vols,depths] = support_depth_search( stable_points,unstable_points,G,cfg);

        if ~isempty(Vs)
            [bestVol,jj] = max(vols);

            best.V = Vs{jj};
            best.bad = bads(jj);
            best.depth = depths(jj);
            best.limit = stable_points;
            best.fallback = true;

            region.used_support_fallback = true;
        end
    end

    if isempty(best.V)

        % Important: do not throw an error here.
        % The expensive sweep data must still be returned and saved.
        warning(['No zero-unstable inner hexahedron was found from the ' 'current sampled grid. Raw Stable/Unstable data will ' 'still be saved for post-processing.']);
        return;
    end

    region.inner_limit_points = best.limit;
    region.inner_vertices = best.V;
    region.inner_volume = bestVol;
    region.unstable_inside_inner = best.bad;
    region.inner_depth = best.depth;
    region.is_valid_inner = (best.bad==0);
    region.used_stable_mask_fallback = region.used_stable_mask_fallback || best.fallback;

end

function Pts = lateral_layer_from_limit_roll(limit_roll,G,depth)

    % At equal (r,phi), sort limit-roll points by vc.
    % Select the depth-th point from the negative/left and positive/right
    % lateral-speed sides.

    Pts = zeros(0,3);
    tol = 1e-12;

    for ir=1:numel(G.r)

        rr = G.r(ir);

        for ip=1:numel(G.phi)

            ph = G.phi(ip);

            idx = abs(limit_roll(:,2)-rr)<=tol & abs(limit_roll(:,3)-ph)<=tol;

            Q = sortrows(limit_roll(idx,:),1);
            n = size(Q,1);

            if n < depth
                continue;
            end

            Pts(end+1,:) = Q(depth,:); %#ok<AGROW>

            j = n-depth+1;

            if j~=depth && j>=1 && j<=n
                Pts(end+1,:) = Q(j,:); %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function Pts = lateral_layer_from_stable_mask(G,stable_mask,depth)

    % Fallback for a discrete grid:
    % At each fixed (r,phi), select the depth-th stable vc point from both
    % sides using the already-computed stable_mask.

    Pts = zeros(0,3);

    for ir=1:numel(G.r)
        for ip=1:numel(G.phi)

            idxVc = find(squeeze(stable_mask(:,ir,ip)));
            n = numel(idxVc);

            if n < depth
                continue;
            end

            iL = idxVc(depth);
            iR = idxVc(n-depth+1);

            Pts(end+1,:) = [G.vc(iL),G.r(ir),G.phi(ip)]; %#ok<AGROW>

            if iR~=iL
                Pts(end+1,:) = [G.vc(iR),G.r(ir),G.phi(ip)]; %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function V = select_ordered_eight_vertices(Pts,G,stable_points)

    % Eight normalized support directions in [vc,r,phi].
    % Returned order:
    % 1 [- - -], 2 [- - +], 3 [- + -], 4 [- + +]
    % 5 [+ - -], 6 [+ - +], 7 [+ + -], 8 [+ + +]

    V = zeros(0,3);

    if size(Pts,1)<8
        return;
    end

    ranges = [ range_nonzero(G.vc), range_nonzero(G.r), range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (Pts-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    chosen = false(size(Pts,1),1);
    V = nan(8,3);

    for k=1:8

        s = signs(k,:);

        % Prefer a point in the corresponding octant.
        oct = true(size(Pts,1),1);

        for j=1:3
            if s(j)<0
                oct = oct & (Q(:,j)<=1e-12);
            else
                oct = oct & (Q(:,j)>=-1e-12);
            end
        end

        score = Q*s(:);
        score(chosen) = -Inf;
        score(~oct) = -Inf;

        [best,idx] = max(score);

        % If the exact octant has no unused point, use the best unused point
        % along the same normalized support direction.
        if ~isfinite(best)
            score = Q*s(:);
            score(chosen) = -Inf;
            [best,idx] = max(score);
        end

        if ~isfinite(best)
            V = zeros(0,3);
            return;
        end

        V(k,:) = Pts(idx,:);
        chosen(idx) = true;
    end

    if size(unique(V,'rows'),1)~=8
        V = zeros(0,3);
    end

end

function [Vcells,badList,volList,depthList] = support_depth_search( stable_points,unstable_points,G,cfg)

    Vcells = {};
    badList = [];
    volList = [];
    depthList = [];

    if size(stable_points,1)<8
        return;
    end

    ranges = [ range_nonzero(G.vc), range_nonzero(G.r), range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (stable_points-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    order = cell(8,1);

    for k=1:8
        score = Q*signs(k,:).';
        [~,order{k}] = sort(score,'descend');
    end

    maxDepth = min( cfg.max_support_depth, max(1,floor(size(stable_points,1)/8)));

    for depth=1:maxDepth

        used = false(size(stable_points,1),1);
        V = nan(8,3);
        ok = true;

        for k=1:8

            ord = order{k};
            pick = [];

            for jj=depth:numel(ord)

                ii = ord(jj);

                if ~used(ii)
                    pick = ii;
                    break;
                end
            end

            if isempty(pick)
                ok = false;
                break;
            end

            V(k,:) = stable_points(pick,:);
            used(pick) = true;
        end

        if ~ok || size(unique(V,'rows'),1)~=8
            continue;
        end

        [vol,okVol] = convex_volume(V);

        if ~okVol
            continue;
        end

        bad = nnz(points_inside_convex_hull( unstable_points,V,cfg.hull_inside_tol));

        fprintf('  Support fallback depth=%2d | volume=%10.6g | bad=%d\n', depth,vol,bad);

        if bad==0
            Vcells{end+1,1} = V; %#ok<AGROW>
            badList(end+1,1) = bad; %#ok<AGROW>
            volList(end+1,1) = vol; %#ok<AGROW>
            depthList(end+1,1) = depth; %#ok<AGROW>
        end
    end

end

function y = range_nonzero(x)

    y = max(x)-min(x);

    if ~(isfinite(y) && y>0)
        y = 1;
    end

end

function V = select_eight_hexahedron_vertices(Pts,G)

    V = zeros(0,3);

    if size(Pts,1)<8

        return;

    end

    scale = [max(max(G.vc)-min(G.vc),eps), max(max(G.r)-min(G.r),eps), max(max(G.phi)-min(G.phi),eps)];

    vcMid = median(Pts(:,1));

    leftIdx = find(Pts(:,1)<=vcMid);

    rightIdx = find(Pts(:,1)>=vcMid);

    if numel(leftIdx)<4 || numel(rightIdx)<4

        % Fallback: split by sorted lateral speed into two overlapping halves.

        [~,ord] = sort(Pts(:,1));

        n = numel(ord);

        leftIdx = ord(1:max(4,ceil(n/2)));

        rightIdx = ord(min(n-3,floor(n/2)+1):n);

    end

    VL = select_four_rphi_extremes(Pts(leftIdx,:),scale);

    VR = select_four_rphi_extremes(Pts(rightIdx,:),scale);

    V = unique([VL;VR],'rows','stable');

    if size(V,1)<8

        % Fill deterministically using points farthest from the current center.

        c = median(Pts,1);

        d2 = sum(((Pts-c)./scale).^2,2);

        [~,ord] = sort(d2,'descend');

        for ii=ord(:).'

            p = Pts(ii,:);

            if ~ismember(p,V,'rows')

                V(end+1,:) = p; %#ok<AGROW>

            end

            if size(V,1)>=8

                break;

            end

        end

    end

    if size(V,1)>8

        V = reduce_to_max_volume_eight(V);

    end

end

function V4 = select_four_rphi_extremes(Pts,scale)

    V4 = zeros(0,3);

    if isempty(Pts)

        return;

    end

    c = median(Pts,1);

    Q = (Pts-c)./scale;

    dirs = [-1 -1; -1 +1; +1 -1; +1 +1];

    chosen = zeros(0,1);

    for k=1:4

        score = Q(:,2)*dirs(k,1) + Q(:,3)*dirs(k,2);

        [~,ord] = sort(score,'descend');

        pick = [];

        for jj=ord(:).'

            if ~ismember(jj,chosen)

                pick=jj;

                break;

            end

        end

        if isempty(pick)

            pick=ord(1);

        end

        chosen(end+1,1)=pick; %#ok<AGROW>

    end

    V4 = unique(Pts(chosen,:),'rows','stable');

    if size(V4,1)<4

        d2 = Q(:,2).^2 + Q(:,3).^2;

        [~,ord] = sort(d2,'descend');

        for jj=ord(:).'

            p=Pts(jj,:);

            if ~ismember(p,V4,'rows')

                V4(end+1,:)=p; %#ok<AGROW>

            end

            if size(V4,1)>=4

                break;

            end

        end

    end

end

function V8 = reduce_to_max_volume_eight(V)

    % Normally V already has exactly 8 points.  This deterministic fallback

    % removes points one-by-one while retaining the largest convex volume.

    V8=V;

    while size(V8,1)>8

        bestVol=-Inf;

        bestIdx=1;

        for i=1:size(V8,1)

            T=V8;

            T(i,:)=[];

            [vol,ok]=convex_volume(T);

            if ok && vol>bestVol

                bestVol=vol;

                bestIdx=i;

            end

        end

        V8(bestIdx,:)=[];

    end

end

function inside = points_inside_convex_hull(P,V,tol)

    inside=false(size(P,1),1);

    if isempty(P) || size(V,1)<4

        return;

    end

    try

        K=convhulln(V);

    catch

        return;

    end

    c=mean(V,1);

    for i=1:size(P,1)

        p=P(i,:);

        isIn=true;

        for f=1:size(K,1)

            tri=V(K(f,:),:);

            n=cross(tri(2,:)-tri(1,:),tri(3,:)-tri(1,:));

            nn=norm(n);

            if nn<eps

                continue;

            end

            n=n/nn;

            if dot(n,c-tri(1,:))>0

                n=-n;

            end

            if dot(n,p-tri(1,:))>tol

                isIn=false;

                break;

            end

        end

        inside(i)=isIn;

    end

end

function [vol,ok] = convex_volume(V)

    vol=NaN;

    ok=false;

    if size(V,1)<4 || rank(V-mean(V,1))<3

        return;

    end

    try

        [~,vol]=convhulln(V);

        ok=isfinite(vol) && vol>0;

    catch

        vol=NaN;

        ok=false;

    end

end

