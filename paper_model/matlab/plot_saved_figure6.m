% Plot previously saved cases without re-simulating.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(root);
source=fullfile(root,'results_figure6','figure6_comparison.mat');
assert(exist(source,'file')==2,'Run run_figure6 first, then replot here.');
L=load(source,'cases');
paper_plot_region(L.cases,true);
