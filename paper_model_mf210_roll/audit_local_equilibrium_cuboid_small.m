function audit_local_equilibrium_cuboid_small()
% Compare filtered histogram search with exhaustive boxes on nonuniform grids.
  started=tic(); rng(27182,'twister'); shapes=[3 3 3;4 4 3;4 3 4];
  positive=0; empty=0; cases=0;
  for shape_index=1:size(shapes,1)
    shape=shapes(shape_index,:);
    G=struct('vc',cumsum(0.1+1.4*rand(shape(1),1)), ...
             'r',cumsum(0.1+1.4*rand(shape(2),1)), ...
             'phi',cumsum(0.1+1.4*rand(shape(3),1)));
    for run=1:40
      mask=rand(shape)<0.87;
      lo=[G.vc(1),G.r(1),G.phi(1)]; hi=[G.vc(end),G.r(end),G.phi(end)];
      anchor=lo+rand(1,3).*(hi-lo);
      if mod(run,8)==0, anchor=lo-[5e-13,0,0]; end
      if mod(run,8)==1, anchor=lo-[2e-12,0,0]; end
      if mod(run,8)==2, anchor=[G.vc(2),G.r(2),G.phi(2)]; end
      [is_positive,~]=compare_one(mask,G,anchor);
      cases=cases+1; positive=positive+is_positive; empty=empty+~is_positive;
    end
  end
  % A larger disconnected box excludes the anchor. The smaller anchored box
  % must win; this detects accidental reuse of an unrestricted maximum.
  G=struct('vc',[0;0.3;1.2;2;3;4.5], 'r',[-2;-0.5;0;0.8;2], 'phi',[-0.7;0;0.4;1]);
  mask=false(6,5,4); mask(1:3,:,:)=true; mask(5:6,2:4,:)=true;
  [is_positive,result]=compare_one(mask,G,[3.8,0.1,0.2]);
  assert(is_positive && isequal(result.grid_indices_1based,[5 6 2 4 1 4]));
  cases=cases+1; positive=positive+1;
  % Equal-volume disconnected anchored candidates verify complete ties.
  G=struct('vc',[0;1;2], 'r',[0;1;2], 'phi',[0;1;2]);
  mask=true(3,3,3); mask(3,3,:)=false; mask(1,1,:)=false;
  [is_positive,result]=compare_one(mask,G,[1,1,1]);
  assert(is_positive && result.maximum_volume_tie_count>1);
  cases=cases+1; positive=positive+1;
  report=struct('method','Independent exhaustive all-box search with equilibrium containment', ...
      'rng_seed',27182,'rng','twister','random_mask_shapes',shapes, ...
      'random_cases',120,'directed_cases',2,'total_cases',cases, ...
      'positive_cases',positive,'empty_cases',empty,'nonuniform_grids',true, ...
      'boundary_tolerance_cases',true,'disconnected_off_anchor_maximum_case',true, ...
      'multiple_maximum_ties_case',true,'all_maximum_volumes_match',true, ...
      'all_ties_match',true,'lexicographic_selections_match',true,'all_checks_pass',true, ...
      'runtime_s',toc(started));
  here=fileparts(mfilename('fullpath'));
  fid=fopen(fullfile(here,'local_cuboid_small_grid_audit.json'),'w'); fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
  fprintf('local brute comparison: cases=%d, positive=%d, empty=%d, pass=1, runtime_s=%.3f\n', ...
      cases,positive,empty,report.runtime_s);
end

function [positive,result]=compare_one(mask,G,anchor)
  [best,ties]=brute_force(mask,G,anchor); result=build_local_equilibrium_cuboid(mask,G,anchor);
  positive=~isempty(ties);
  assert(logical(result.valid)==positive,'Availability mismatch');
  if ~positive, return; end
  assert(abs(result.volume-best)<1e-12*max(1,abs(best)),'Maximum volume mismatch');
  assert(isequal(result.maximum_volume_ties_grid_bound_indices_zero_based,ties),'Maximum ties mismatch');
  assert(isequal(result.grid_bound_indices_zero_based,ties(1,:)),'Lexicographic selection mismatch');
  assert(result.contains_equilibrium,'Selected box misses anchor');
end

function [best,ties]=brute_force(mask,G,anchor)
  shape=size(mask); best=-1; ties=zeros(0,6); coordinates={G.vc,G.r,G.phi};
  for v0=1:shape(1)-1
    for v1=v0+1:shape(1)
      for r0=1:shape(2)-1
        for r1=r0+1:shape(2)
          for p0=1:shape(3)-1
            for p1=p0+1:shape(3)
              lower=[G.vc(v0),G.r(r0),G.phi(p0)]; upper=[G.vc(v1),G.r(r1),G.phi(p1)];
              if any(anchor<lower-1e-12) || any(anchor>upper+1e-12), continue; end
              subset=mask(v0:v1,r0:r1,p0:p1); if ~all(subset(:)), continue; end
              volume=prod(upper-lower); bounds=[v0 v1 r0 r1 p0 p1]-1;
              tolerance=1e-13*max([1 abs(best) abs(volume)]);
              if volume>best+tolerance, best=volume; ties=bounds;
              elseif abs(volume-best)<=tolerance, ties(end+1,:)=bounds; end %#ok<AGROW>
            end
          end
        end
      end
    end
  end
  ties=sortrows(unique(ties,'rows'),1:6);
end
