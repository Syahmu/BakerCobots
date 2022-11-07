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

% render the environment
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
%UR3.model.animate(UR3StartingQ);

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







p1Star = [662; 362]; p2Star=[512; 62]; p3Star=[512; 962]; p4Star = [662; 662];
pStar = [p2Star p3Star] %p2Star p3Star p4Star]%p1Star p2Star p3Star p4Star] %These points are the final location for the camera's image view

%Cartesian coordinates of the targets corners
%Actual coodrinates of the target points

P1 = [0; -1; 0.5]; P2 = [0; -1; 1]%1.8; 0.25; 1.25]; P3 = [1.8; 0.25; 0.75]; P4 = [1.8; -0.25; 0.75];

%P = [P1 P2] %P2 P3 P4]

handleFkine = ovenDoor.model.fkine(ovenDoor.model.getpos())
handlePosition1 = [handleFkine(1,4)+0.1;handleFkine(2,4);handleFkine(3,4)]
handlePosition2 = [handleFkine(1,4)-0.1;handleFkine(2,4);handleFkine(3,4)]         
P = [handlePosition1 handlePosition2]

%IIWA  initial joint angles

%q0 = [pi/2 -pi/3 -pi/3 -pi/6 0 0];
initQ = [pi/2,0,0,0,0,-pi/2,0]

%perspective Camera (CentralCamera) 
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512], 'fps', 25, 'name', 'mycamera');
% with focal length of 0.08, 
% pixel size of 10e‐5, 
% a resolution of 1024 x 1024 with the centre point exactly in the middle of image plane, 
% which gets images at 25fps.

%Locate the camera at the position of the end effector

campos = LBR.model.fkine(initQ)%UR10.model.fkine(q0)

%Define the visual servoing gain (0 < lambda < 1) and 
lambda = 0.9

%a value for the depth. points are roughly at 1.8m from the camera
depth = 1;
fps = 25
axis vis3d

%display the iiwa @ q0

LBR.model.plot(initQ);

%display the camera in 3d at the end effector location

cam.plot_camera('Tcam',campos);

%plot the points in 3d

plot_sphere(P , 0.05, 'b');




%display the target in the camera's image view
hold on
cam.plot([pStar], '*');



cam.plot(P, 'Tcam', campos, 'o'); %display the current view using the projection of 3d points
%cam.plot(P2, 'Tcam', campos, 'o');
%cam.plot(P3, 'Tcam', campos, 'o');
%cam.plot(P4, 'Tcam', campos, 'o');
hold off



Q = initQ
variablecampos = [campos];

for i = 1:400 %looping through each
%view of the camera through projecting the 3D points
uv = cam.plot(P, 'Tcam', variablecampos, 'o');


%calculate the error in the image
e = pStar - uv

%current Image Jacobian

J = cam.visjac_p(uv, depth);




%desired velocity of end effector (camera velocity)

v = lambda * pinv(J) * e(:);

%calculate the joint velocities of the UR10 (compute robots jacobian wrt to
%end effector frame)

Jacob = LBR.model.jacobn(Q);

JointVel = transpose(Jacob)*v;


%joint velocities -180 <= x <= 180
if JointVel >= pi
    JointVel = pi
end
if JointVel <= -pi
    JointVel = -pi
end


%calculate joint displacements

DeltaT = 1/fps;
newQ = Q + transpose(DeltaT * JointVel); %joint displacement

%Apply Displacement to UR10 + update camera location

LBR.model.animate(newQ);



 
cam.plot_camera('Tcam',variablecampos);
%Display UR10 + current camera location
cam.plot([P], 'Tcam', variablecampos, 'o');

variablecampos = LBR.model.fkine(newQ);
Q = newQ; %newQ = Q for loop repition
cam.clf;
%When the desired image location has been reached or the maximum number 
%of steps have been reach, finish the loop
if abs(e) <= 1
    break;
end
%Plot points trajectory in the image, camera position, orientation and 
%velocities and joint angles and velocities 

end


%insert inputs in the class function
control = Control(LBR,tray,GripperState,LBRGripper1,LBRGripper2,ovenDoor, objectArray, avoidCollisions, resolvedMotionRateControl);
%then run the function within the class
%control.Start();
