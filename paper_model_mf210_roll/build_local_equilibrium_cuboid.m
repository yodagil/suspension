function result = build_local_equilibrium_cuboid(stable_mask,G,equilibrium_point)
%BUILD_LOCAL_EQUILIBRIUM_CUBOID Exact largest all-stable box containing equilibrium.
% equilibrium_point = [vc,r,phi] is an anchor to contain, not a prescribed
% center. The same objective and tolerances apply independently at every
% steering angle. No aspect ratios, nesting, angle factors, or roll caps.
%
% Enumerate every distinct phi-bound pair containing the anchor, intersect
% the stable masks through that interval, and enumerate maximal all-one
% vc-r rectangles with a monotone histogram stack. Every positive-volume
% all-one rectangle containing the anchor can expand to a maximal rectangle
% that still contains it. Thus the search gives the global maximum among
% grid-aligned all-stable cuboids containing the specified equilibrium.
% Physical coordinate spans determine volume; nonuniform grids are allowed.
%
% Containment tolerance: 1e-12. Volume ties use
% 1e-13*max(1,abs(best),abs(candidate)); select the lexicographically first
% [vc_lo,vc_hi,r_lo,r_hi,phi_lo,phi_hi] grid-index tuple and save every tie.
% No statement is made about unsampled states inside the cuboid. The caller
% must separately establish equilibrium local stability and vehicle limits.
  stable_mask = logical(stable_mask);
  coordinates = {G.vc(:),G.r(:),G.phi(:)};
  expected_shape = cellfun(@numel,coordinates);
  assert(isequal(size(stable_mask),expected_shape), 'Mask/grid size mismatch');
  for k=1:3
    assert(all(isfinite(coordinates{k})) && all(diff(coordinates{k})>0), ...
           'Grid coordinates must be finite and strictly increasing');
  end
  equilibrium_point=double(equilibrium_point(:).');
  assert(numel(equilibrium_point)==3 && all(isfinite(equilibrium_point)), ...
         'Equilibrium point must contain three finite [vc,r,phi] values');
  containment_tolerance=1e-12;
  nv=expected_shape(1); nr=expected_shape(2); np=expected_shape(3);
  best_volume=-1; best_bounds=[]; ties=zeros(0,6);
  slabs=0; rectangles=0;
  for p0=1:np-1
    if coordinates{3}(p0)>equilibrium_point(3)+containment_tolerance, continue; end
    intersection=true(nv,nr);
    for p1=p0:np
      intersection = intersection & stable_mask(:,:,p1);
      if p1==p0 || coordinates{3}(p1)<equilibrium_point(3)-containment_tolerance, continue; end
      slabs=slabs+1;
      heights=zeros(1,nr);
      for v1=1:nv
        good=intersection(v1,:);
        heights(~good)=0; heights(good)=heights(good)+1;
        stack_start=zeros(1,nr); stack_height=zeros(1,nr); top=0;
        for r_end=1:nr+1
          if r_end<=nr, height=heights(r_end); else, height=0; end
          start=r_end;
          while top>0 && stack_height(top)>height
            r0=stack_start(top); h=stack_height(top); top=top-1;
            start=r0; r1=r_end-1;
            if h<2 || r1<=r0, continue; end
            v0=v1-h+1;
            if ~(coordinates{1}(v0)<=equilibrium_point(1)+containment_tolerance && ...
                 coordinates{1}(v1)>=equilibrium_point(1)-containment_tolerance && ...
                 coordinates{2}(r0)<=equilibrium_point(2)+containment_tolerance && ...
                 coordinates{2}(r1)>=equilibrium_point(2)-containment_tolerance)
              continue;
            end
            volume=(coordinates{1}(v1)-coordinates{1}(v0)) * ...
                   (coordinates{2}(r1)-coordinates{2}(r0)) * ...
                   (coordinates{3}(p1)-coordinates{3}(p0));
            rectangles=rectangles+1;
            bounds=[v0,v1,r0,r1,p0,p1]-1;
            tolerance=1e-13*max([1,abs(best_volume),abs(volume)]);
            if volume>best_volume+tolerance
              best_volume=volume; best_bounds=bounds; ties=bounds;
            elseif abs(volume-best_volume)<=tolerance
              ties(end+1,:)=bounds; %#ok<AGROW>
              if lexicographic_less(bounds,best_bounds), best_bounds=bounds; end
            end
          end
          if height>0 && (top==0 || stack_height(top)<height)
            top=top+1; stack_start(top)=start; stack_height(top)=height;
          end
        end
      end
    end
  end
  if isempty(best_bounds)
    result=struct('valid',false,'volume',NaN,'vertices',zeros(0,3),'faces',zeros(0,4), ...
      'bounds',NaN(3,2),'grid_indices_1based',zeros(0,6),'all_ties_grid_indices_1based',zeros(0,6), ...
      'grid_bound_indices_zero_based',zeros(0,6),'maximum_volume_tie_count',0, ...
      'maximum_volume_ties_grid_bound_indices_zero_based',zeros(0,6), ...
      'equilibrium_anchor_vc_r_phi',equilibrium_point,'equilibrium_containment_required',true, ...
      'containment_tolerance',containment_tolerance, ...
      'positive_equilibrium_containing_roll_intervals_tested',slabs, ...
      'positive_equilibrium_containing_rectangles_evaluated',rectangles, ...
      'unavailable_reason','No positive-volume all-stable grid cuboid contains equilibrium');
    return;
  end
  ties=sortrows(unique(ties,'rows'),1:6);
  lower=zeros(1,3); upper=zeros(1,3);
  for k=1:3
    lower(k)=coordinates{k}(best_bounds(2*k-1)+1);
    upper(k)=coordinates{k}(best_bounds(2*k)+1);
  end
  vertices=zeros(8,3); index=0;
  for iv=1:2
    for ir=1:2
      for ip=1:2
        index=index+1;
        bounds=[lower;upper];
        vertices(index,:)=[bounds(iv,1),bounds(ir,2),bounds(ip,3)];
      end
    end
  end
  tie_roll_bounds=[coordinates{3}(ties(:,5)+1),coordinates{3}(ties(:,6)+1)];
  tie_roll_spans=tie_roll_bounds(:,2)-tie_roll_bounds(:,1);
  b=best_bounds+1;
  subset=stable_mask(b(1):b(2),b(3):b(4),b(5):b(6));
  result=struct('valid',true,'grid_bound_indices_zero_based',best_bounds, ...
       'grid_indices_1based',best_bounds+1,'all_ties_grid_indices_1based',ties+1, ...
       'bounds',[lower(:),upper(:)],'faces',[1 2 4 3;5 7 8 6;1 5 6 2;3 4 8 7;1 3 7 5;2 6 8 4], ...
       'lower_bounds',lower,'upper_bounds',upper,'coordinate_spans',upper-lower, ...
       'roll_bounds',[lower(3),upper(3)],'roll_span',upper(3)-lower(3), ...
       'coordinate_centers',(upper+lower)/2,'roll_center',(upper(3)+lower(3))/2, ...
       'vertices',vertices,'inner_vertices',vertices,'volume',best_volume,'inner_volume',best_volume, ...
       'is_valid_inner',true,'equilibrium_anchor_vc_r_phi',equilibrium_point, ...
       'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
       'containment_tolerance',containment_tolerance, ...
       'contains_equilibrium',all(equilibrium_point>=lower-containment_tolerance) && ...
                              all(equilibrium_point<=upper+containment_tolerance), ...
       'all_grid_nodes_in_cuboid_stable',all(subset(:)), ...
       'grid_nodes_in_cuboid',numel(subset), ...
       'maximum_volume_tie_count',size(ties,1), ...
       'maximum_volume_ties_grid_bound_indices_zero_based',ties, ...
       'maximum_volume_ties_roll_bounds',tie_roll_bounds, ...
       'maximum_volume_ties_roll_span_range',[min(tie_roll_spans),max(tie_roll_spans)], ...
       'positive_equilibrium_containing_roll_intervals_tested',slabs, ...
       'positive_equilibrium_containing_rectangles_evaluated',rectangles, ...
       'global_maximum_among_equilibrium_containing_grid_cuboids',true);
end

function answer=lexicographic_less(a,b)
  answer=false;
  for k=1:numel(a)
    if a(k)<b(k), answer=true; return; end
    if a(k)>b(k), return; end
  end
end
