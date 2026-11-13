function executable=hmc_build_native(compiler)
%HMC_BUILD_NATIVE Build independent HMC C++17/OpenMP accelerator.
if nargin<1,compiler='g++';end
root=fileparts(mfilename('fullpath'));executable=fullfile(root,'native','hmc_native');
if ispc,executable=[executable '.exe'];end
src=fullfile(root,'native','hmc_native.cpp');
assert(~any(compiler=='"')&&~any(src=='"')&&~any(executable=='"'),'Invalid quoted path');
cmd=sprintf('"%s" -O3 -std=c++17 -fopenmp "%s" -o "%s"',compiler,src,executable);
[status,msg]=system(cmd);assert(status==0,'Native compilation failed: %s',msg);
fprintf('Built independent HMC engine: %s\n',executable);
end
