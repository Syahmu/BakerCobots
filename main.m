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
%UR3 = UR3robot;
LBR = iiwa;

trayPose.position = [0,-0.8863,ovenHeight];

trayHeight = 0.0268;

%% render the environment
setScene = SetScene(LBR,trayPose);
camlight;
objectArray = setScene.BuildEnvironment();

%loading the oven door
ovenDoor = OvenDoorBot;
doorStartingQ = 0;
ovenDoor.model.base = ovenDoor.model.base * transl(-0.8,-0.82,0);
ovenDoor.model.animate(doorStartingQ)

%setting reasonable starting pose for robot
iiwaStartingQ = [0,0,0,0,0,0,0];
LBR.model.base = LBR.model.base * transl(0,0,0.79);
LBR.model.animate(iiwaStartingQ)

avoidCollisions = true;
resolvedMotionRateControl = true;

%insert inputs in the class function
control = Control(LBR, ovenDoor, objectArray, trayPose, avoidCollisions, resolvedMotionRateControl);
%then run the function within the class
control.Start();
