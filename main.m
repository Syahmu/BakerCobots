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

%loading the oven door
tray = TrayBot;
tray.model.base = tray.model.base * transl(0,-0.95,1.3);
tray.model.animate(0);

%loading the oven door
ovenDoor = OvenDoorBot;
doorStartingQ = 0;
ovenDoor.model.base = ovenDoor.model.base * transl(-1.5,-0.82,0);
ovenDoor.model.animate(doorStartingQ);

%setting reasonable starting pose for robot
iiwaStartingQ = zeros(1,7);
LBR.model.base = LBR.model.base * transl(-0.3,-0.3,0.79); % x z y
LBR.model.animate(iiwaStartingQ);

UR3StartingQ = zeros(1,7);
UR3.model.base = UR3.model.base * transl(0,1.3,0.79); % x z y
UR3.model.animate(UR3StartingQ);

avoidCollisions = true;
resolvedMotionRateControl = true;

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

%insert inputs in the class function
control = Control(LBR,tray,GripperState,LBRGripper1,LBRGripper2,ovenDoor, objectArray, avoidCollisions, resolvedMotionRateControl);
%then run the function within the class
control.Start();
