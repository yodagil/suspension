function out=cdc_run_points(points,Q,c,T)
% Combine explicitly listed hidden-state variants, not all hidden states.
variants=cell(size(c.hidden_variants,1),1);
for j=1:numel(variants)
 X=cdc_fullcar_model('lift',points,Q,c,T,c.hidden_variants(j,:));
 variants{j}=cdc_research_simulate(X,Q,c,T);
end
out=variants{1};if numel(variants)==1,return;end
names=cdc_research_fields();boolean=[1:9 26];minimum=[12 14 15 16 17 21 22 23 25];
for iv=2:numel(variants)
 b=variants{iv};
 for j=1:numel(names)
  key=names{j};
  if any(j==boolean),out.(key)=double(logical(out.(key))&logical(b.(key)));
  elseif j==24,out.reason=double(bitor(uint16(out.reason),uint16(b.reason)));
  elseif any(j==minimum),out.(key)=min(out.(key),b.(key));
  else,out.(key)=max(out.(key),b.(key));end
 end
end
out.variant_results=variants;out.final_state=[];out.trace=[];
end
