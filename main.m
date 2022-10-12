%BakerCobot
%SID 14034882 - Ahmad Syahmi Mohd Nasir

clear
clc
clf

hold on
%% Initiate the 2 robots

%set some parameters
tableHeight = 0.79;

%Load in robot arms
ur3 = UR3;
LBR = iiwa;

SetScene(tableHeight, ur3, LBR);