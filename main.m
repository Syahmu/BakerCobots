%BakerCobot
%SID 14034882 - Ahmad Syahmi Mohd Nasir

clear
clc
clf

hold on
%% Initiate the 2 robots

%set some parameters
tableHeight = 0.79;
ovenHeight = 0.9135;

%Load in robot arms
UR3 = UR3robot;
LBR = iiwa;

%% render the environment
setScene = SetScene();
camlight;
objectArray = setScene.BuildEnvironment();

%loading the Tray
tray = TrayBot;
tray.model.base = tray.model.base * transl(0,-1.2,-0.95) * troty(pi);
tray.model.animate(0);

bread = BreadBot;
bread.model.base = tray.model.fkine(tray.model.getpos()) * trotx(pi/2);
bread.model.animate(0);

%loading the oven door
ovenDoor = OvenDoorBot;
doorStartingQ = 0;
ovenDoor.model.base = ovenDoor.model.base * transl(-1.5,-0.82,0);
ovenDoor.model.animate(doorStartingQ);

%setting reasonable starting pose for robot
iiwaStartingQ = zeros(1,7);
LBR.model.base = LBR.model.base * transl(-0.3,-0.3,0.79); % x z y
LBR.model.animate(iiwaStartingQ);

UR3StartingQ = [0,-pi/2,0,0,0,0,0];
UR3.model.base = UR3.model.base * transl(-0.3,1.1,0.79); % x z y
UR3.model.animate(UR3StartingQ);

avoidCollisions = false;

open = 70;
close = 90;
GripperState = open; %gripper angles 70-90

% gripper : note that its z axis is swapped with y
LBRGripper1 = Gripper;
LBRGripper1.model.base = LBR.model.fkine(LBR.model.getpos()) * trotx(pi/2);
LBRGripper1.model.animate(deg2rad(GripperState));
LBRGripper2 = Gripper;
LBRGripper2.model.base = LBR.model.fkine(LBR.model.getpos()) * trotx(pi/2) * troty(pi);
LBRGripper2.model.animate(deg2rad(GripperState));

view(2);

%insert inputs in the class function
control = Control(LBR,UR3, tray,bread,GripperState,LBRGripper1,LBRGripper2,ovenDoor, objectArray, avoidCollisions);
%then run the function within the class
control.Start();
