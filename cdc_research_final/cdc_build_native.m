function executable=cdc_build_native(compiler)
% Optional accelerator. Requires a C++17 compiler with OpenMP (e.g. g++).
% Source only; original hard-coded-950 native binary must NOT be reused.
if nargin<1,compiler='g++';end
root=fileparts(mfilename('fullpath'));executable=fullfile(root,'native','cdc_research_native');
if ispc,executable=[executable '.exe'];end
src=fullfile(root,'native','cdc_research_native.cpp');
assert(~any(compiler=='"')&&~any(src=='"')&&~any(executable=='"'),'Invalid quoted path.');
cmd=sprintf('"%s" -O3 -std=c++17 -fopenmp "%s" -o "%s"',compiler,src,executable);
[status,msg]=system(cmd);assert(status==0,'Native compilation failed: %s\nUse o.engine=''mcode'' without a compiler.',msg);
fprintf('Built %s\n',executable);
end
