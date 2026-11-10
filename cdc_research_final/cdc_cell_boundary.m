function xy=cdc_cell_boundary(v,r,cm)
% Exact exposed edges of the union of accepted grid cells; no convex hull.
assert(isequal(size(cm),[numel(v)-1,numel(r)-1]),'Cell/grid size mismatch');
xy=zeros(0,2);[nv,nr]=size(cm);
for i=1:nv
 for j=1:nr
  if ~cm(i,j),continue;end
  if i==1||~cm(i-1,j),xy=[xy;v(i) r(j);v(i) r(j+1);NaN NaN];end %#ok<AGROW>
  if i==nv||~cm(i+1,j),xy=[xy;v(i+1) r(j);v(i+1) r(j+1);NaN NaN];end %#ok<AGROW>
  if j==1||~cm(i,j-1),xy=[xy;v(i) r(j);v(i+1) r(j);NaN NaN];end %#ok<AGROW>
  if j==nr||~cm(i,j+1),xy=[xy;v(i) r(j+1);v(i+1) r(j+1);NaN NaN];end %#ok<AGROW>
 end
end
end
