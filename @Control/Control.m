classdef Control < handle
    %CONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        LBR;
        ovenDoor;
        oven;
        table;
        shelf;
        moveVelocity;
        collisionPlot;
        trajectoryPlot;
        avoidCollisions;
        trajectorySteps;
        errorMax;
        objectPlot;
        objectPosition;
        qMatrix;
        UR3qMatrix
        endQ;
        collisionObjects;
        GripperState;
        LBRGripper1;
        LBRGripper2;
        tray;
        bread;
        UR3;
    end
    
    methods
        
        function obj = Control(LBR,UR3,tray,bread,GripperState,LBRGripper1,LBRGripper2,ovenDoor,objectArray, avoidCollisions)
            %CONTROL Construct an instance of this class
            %   Detailed explanation goes here
            obj.LBR = LBR;
            obj.UR3 = UR3;
            obj.tray = tray;
            obj.bread = bread;
            obj.GripperState = GripperState;
            obj.LBRGripper1 = LBRGripper1;
            obj.LBRGripper2 = LBRGripper2;
            obj.ovenDoor = ovenDoor;
            obj.oven = objectArray{1};
            obj.table = objectArray{2};
            obj.shelf = objectArray{3};
            obj.avoidCollisions = avoidCollisions;
            obj.trajectorySteps = 100;
            obj.errorMax = 1;
            obj.moveVelocity = 2;
        end
        
        %% Start
        function Start(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %obj.MoveToHandle();
            obj.OpenDoor();
            obj.MoveToTray();
            obj.MoveTrayToTable();
            obj.MoveToBread();
            obj.MoveBreadToShelf();
        end
        %% Move To Handle
        function MoveToHandle(obj)
            handleFkine = obj.ovenDoor.model.fkine(obj.ovenDoor.model.getpos());
            handlePosition = [handleFkine(1,4),handleFkine(2,4),handleFkine(3,4)];
            
            point = handlePosition;
            
            pStar = [500;500] %target point is camera centre (pixel coordinates)
            initialQ = obj.LBR.model.getpos() %Initial Q starting position
            %which i have changed to [pi/2,0,0,0,0,-pi/2,0] in the main function for camera roughly facing oven
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ... %perspective Camera (CentralCamera)
                'resolution', [1024 1024], 'centre', [512 512], 'fps', 25, 'name', 'mycamera');
            % with focal length of 0.08,
            % pixel size of 10e???5,
            % a resolution of 1024 x 1024 with the centre point exactly in the middle of image plane,
            % which gets images at 25fps.
            
            campos = obj.LBR.model.fkine(initialQ) %end effector position/camera position
            lambda = 0.5; % visual servoing gain (0 < lambda < 1), can change to make faster/slower
            depth = 1.2; %distance from camera to points (Needs to be roughly defined)
            fps  = 25 %value can be chnaged for faster output
            axis vis3d
            
            cam.plot_camera('Tcam',campos); %display the camera in 3d at the end effector location
            plot_sphere(point , 0.1, 'b') %plot the points in 3d
            cam.plot(pStar, '*'); %display the target in the camera's image view
            cam.plot(point, 'Tcam', campos, 'o');%display the current view using the projection of 3d points
            
            Q = initialQ;
            variablecampos = campos;
            
            for i = 1:200
                
                uv = cam.plot(point, 'Tcam', variablecampos, 'o'); %view of the camera through projecting the 3D points
                e = pStar - uv %calculate the error in the image
                J = cam.visjac_p(uv, depth); %current Image Jacobian
                v = lambda * pinv(J) * e(:); %desired velocity of end effector (camera velocity)
                
                Jacob = UR10.model.jacobn(Q);
                JointVel = Jacob*v; %joint velocities
                
                if JointVel >= pi % -180 <= joint velocities <= 180
                    JointVel = pi
                end
                if JointVel <= -pi
                    JointVel = -pi
                end
                
                DeltaT = 1/fps;
                newQ = Q + transpose(DeltaT * JointVel); %joint displacement
                
                %Apply Displacement + update camera location
                obj.LBR.model.animate(newQ);
                cam.plot_camera('Tcam',variablecampos);
                
                %Display IIWA + current camera location
                cam.plot(point, 'Tcam', variablecampos, 'o');
                variablecampos = obj.LBR.model.fkine(newQ); %updating variablecampos for loop
                
                Q = newQ; %Q = newQ for loop repition
                
                if abs(e) <= 1
                    break;
                end
            end
            
            %obj.MoveEndEffectorToPoint(point,obj.moveVelocity,0);
        end
        
        %% Open Door
        function OpenDoor(obj)
            for i = 0:-0.025:-0.7
                obj.ovenDoor.model.animate(i);
                handleFkine = obj.ovenDoor.model.fkine(obj.ovenDoor.model.getpos());
                handlePosition = [handleFkine(1,4),handleFkine(2,4),handleFkine(3,4)];
                obj.MoveEndEffectorToPoint(handlePosition,obj.moveVelocity,0,0);
            end
            obj.avoidCollisions = false;
            obj.collisionObjects = {};
        end
        
        %% Move To Tray
        function MoveToTray(obj)
            trayPosition = [0,-0.95,1.2];
            obj.MoveEndEffectorToPoint(trayPosition,obj.moveVelocity,0,0);
        end
        
        
        %% Move Tray To Table
        function MoveTrayToTable(obj)
            moveTray = true;
            point = [0,-0.9,1.2];
            facing = 1;
            obj.MoveEndEffectorToPoint(point,obj.moveVelocity,moveTray,facing);
            
            facing = 2;
            point = [0.6,0,1.2];
            obj.MoveEndEffectorToPoint(point,obj.moveVelocity,moveTray,facing);
            
            facing = 3;
            point = [0,0.4,1.2];
            obj.MoveEndEffectorToPoint(point,obj.moveVelocity,moveTray,facing);
            
            point = [0,0.4,1];
            obj.MoveEndEffectorToPoint(point,obj.moveVelocity,moveTray,facing);
        end
        
        %% Move to bread
        function MoveToBread(obj)
            breadFkine = obj.bread.model.fkine(obj.bread.model.getpos());
            breadPosisition = [breadFkine(1,4),breadFkine(2,4),breadFkine(3,4)];
            obj.UR3MoveEndEffectorToPoint(breadPosisition,1,0,0);
        end
        
        %% Move bread to shelf
        function MoveBreadToShelf(obj)
            moveBread = true;
            point = [0,1.6,1.05];
            obj.UR3MoveEndEffectorToPoint(point,1,moveBread,0);
        end
        
        %% Move End Effector to Point
        function MoveEndEffectorToPoint(obj,point,velocity,moveTray,facing)
            obj.ResolveMotionRateControl(point,velocity,moveTray,facing);
        end
        
        function UR3MoveEndEffectorToPoint(obj,point,velocity,moveBread,facing)
            obj.UR3ResolveMotionRateControl(point,velocity,moveBread,facing);
        end
        
        %% using Jtraj
        function MoveToEndEffectorJtraj(obj,endPoint,moveTray)
            %controls the end effector of the ur3 to its target
            
            % number of refreshrates for the movement of the UR3 (Higher = smoother)
            trajectoryLength = 50;
            
            %find start endeffector through forward kinematics using current joints as input
            startFkine = obj.LBR.model.fkine(obj.LBR.model.getpos());
            
            %finding x y z by ifnding the difference from start to end points. (Filter the fkine to just xyz)
            deltaPosition = [(endPoint(1)-startFkine(1,4)),(endPoint(2)-startFkine(2,4)),(endPoint(3)-startFkine(3,4))];
            
            %Finding the endpoint endeffector - week 1
            endFkine = transl(deltaPosition)*startFkine;
            
            %finding the joint states for the endpoint endeffector through inverse
            %kinematics
            
            %using ikcon becasause it takes consideration of the joint limits in
            %comparison to ikine where plane masking is not needed in this application
            endIkcon = obj.LBR.model.ikcon(endFkine);
            
            %using jtraj
            jointTrajectory = jtraj(obj.LBR.model.getpos(),endIkcon,trajectoryLength);
            
            for trajStep = 1:size(jointTrajectory)
                q = jointTrajectory(trajStep,:);
                
                % Animate robot through a fraction of the total movement
                obj.LBR.model.animate(q);
                obj.LBRGripper1.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2);
                obj.LBRGripper1.model.animate(deg2rad(obj.GripperState));
                obj.LBRGripper2.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2) * troty(pi);
                obj.LBRGripper2.model.animate(deg2rad(obj.GripperState));
                
                if moveTray == true
                    obj.tray.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotz(-pi/2);
                    obj.tray.model.animate(0);
                    obj.bread.model.base = obj.tray.model.fkine(obj.tray.model.getpos()) * trotx(pi/2);
                    obj.bread.model.animate(0);
                end
                
                drawnow();
                
                %confirm end effector location
                if trajStep == size(jointTrajectory,1)
                    disp("Confirm end effector: Press enter to continue")
                    %pause();
                end
                
                pause(0.05);
                
            end
        end
        
        function UR3MoveToEndEffectorJtraj(obj,endPoint,moveBread)
            %controls the end effector of the ur3 to its target

            % number of refreshrates for the movement of the UR3 (Higher = smoother)
            trajectoryLength = 50;
            
            %find start endeffector through forward kinematics using current joints as input
            startFkine = obj.UR3.model.fkine(obj.LBR.model.getpos());
            
            %finding x y z by ifnding the difference from start to end points. (Filter the fkine to just xyz)
            deltaPosition = [(endPoint(1)-startFkine(1,4)),(endPoint(2)-startFkine(2,4)),(endPoint(3)-startFkine(3,4))];
            
            %Finding the endpoint endeffector - week 1
            endFkine = transl(deltaPosition)*startFkine;
            
            %finding the joint states for the endpoint endeffector through inverse
            %kinematics
            
            %using ikcon becasause it takes consideration of the joint limits in
            %comparison to ikine where plane masking is not needed in this application
            endIkcon = obj.UR3.model.ikcon(endFkine);
            
            %using jtraj
            jointTrajectory = jtraj(obj.LBR.model.getpos(),endIkcon,trajectoryLength);
            
            for trajStep = 1:size(jointTrajectory)
                q = jointTrajectory(trajStep,:);
                
                % Animate robot through a fraction of the total movement
                obj.UR3.model.animate(q);
                
                if moveBread == true
                    obj.bread.model.base = obj.UR3.model.fkine(obj.UR3.model.getpos());
                    obj.bread.model.animate(0);
                end
                
                drawnow();
                
                %confirm end effector location
                if trajStep == size(jointTrajectory,1)
                    disp("Confirm end effector: Press enter to continue")
                    %pause();
                end
                
                pause(0.05);
                
            end
        end
        
        %% Resolve Motion Rate Control
        function ResolveMotionRateControl(obj,endPoint,velocity,moveTray,facing)
            % Calculate RMRC trajectory
            obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,facing);
            obj.AnimateTrajectory(obj.qMatrix,moveTray);
            
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
        end
        
        %% UR3 resolve motion rate
        function UR3ResolveMotionRateControl(obj,endPoint,velocity,moveBread,facing)
            % Calculate RMRC trajectory
            obj.UR3ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,facing);
            obj.UR3AnimateTrajectory(obj.UR3qMatrix,moveBread);
            
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
        end
        
        
        %% resolve motion rate calculations
        function ResolveMotionRateControlCalculateTrajectory(obj,endPoint,velocity,facing)
            
            % Delete previous trajectory plot
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
            
            % Calculate distance to point to scale the number of steps for
            % each trajectory
            robotTransform = obj.LBR.model.fkine(obj.LBR.model.getpos);
            startPoint = robotTransform(1:3,4)';
            distanceToEndPoint = norm(endPoint-startPoint);
            
            % Robotics
            % Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
            % 1.1) Set parameters for the simulation
            % mdl_puma560;        % Load robot model
            t = (distanceToEndPoint)/velocity;             % Total time (s)
            deltaT = 0.005;      % Control frequency
            steps = round(t/deltaT,0,'decimals');   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.2;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector 1s for more weighting than 0.1 angular velocities
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            obj.qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            % Modify the rotation of the end effector based on the object position
            if endPoint(1)>0.01
                objectTransform = troty(-pi/2);
            elseif endPoint(1) <0.01
                objectTransform = troty(pi/2);
            else
                objectTransform = eye(4);
            end
            
            if endPoint(2)>0.01
                objectTransform = objectTransform*trotx(-pi/2);
            elseif endPoint(2) <0.01
                objectTransform = objectTransform*trotx(pi/2);
            end
            
            if facing == 1
                objectTransform = eye(4) * trotx(pi/2) * troty(-pi/2);
            end
            if facing == 2
                objectTransform = eye(4) * troty(pi/2);
            end
            if facing == 3
                objectTransform = eye(4) * trotx(-pi/2) * trotz(pi/2);
            end
            
            rpy = tr2rpy(objectTransform);
            
            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1); % Points in x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Points in y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
                
                theta(1,i) = rpy(1);                                        % Roll angle
                theta(2,i) = rpy(2);                                        % Pitch angle
                theta(3,i) = rpy(3);                                        % Yaw angle
            end
            
            q0 = zeros(1,7);                                                            % Initial guess for joint angles
            obj.qMatrix(1,:) = obj.LBR.model.getpos;                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = obj.LBR.model.fkine(obj.qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(abs(Rd*Ra'));                                            % Convert rotation matrix to RPY angles. Gives error as rotation matrix times its transform should equal the indentity matrix
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = obj.LBR.model.jacob0(obj.qMatrix(i,:));                                          % Get Jacobian at current joint state. Jacob gives with respect to base!!!
                m(i) = sqrt(abs(det(J*J')));
                if m(i) < epsilon                                                       % If manipulability is less than given threshold (epsilon was set at top)
                    lambda = (1 - m(i)/epsilon)*5E-2; % If manipubility is insufficient then damping will be < 0
                else
                    lambda = 0; % If manipubility is sufficient then damping will be = 0
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:7                                                             % Loop through joints 1 to 7
                    if obj.qMatrix(i,j) + deltaT*qdot(i,j) < obj.LBR.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif obj.qMatrix(i,j) + deltaT*qdot(i,j) > obj.LBR.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                obj.qMatrix(i+1,:) = obj.qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            Transfrom = obj.LBR.model.fkine(obj.qMatrix(end,:));
            endPosition = Transfrom(1:3,4)';
            
            error = endPosition-endPoint;
            obj.errorMax = max(abs(error));
            
            obj.trajectoryPlot = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);
            
            % Display errors
            disp('End Point Translation Error: ')
            disp(error);
            
            disp('End Point Rotation Error (degrees): ')
            disp(rad2deg(angleError(:,end))');
            
        end
        
        function UR3ResolveMotionRateControlCalculateTrajectory(obj,endPoint,velocity,facing)
            
            % Delete previous trajectory plot
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
            
            % Calculate distance to point to scale the number of steps for
            % each trajectory
            robotTransform = obj.UR3.model.fkine(obj.UR3.model.getpos);
            startPoint = robotTransform(1:3,4)';
            distanceToEndPoint = norm(endPoint-startPoint);
            
            % Robotics
            % Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
            % 1.1) Set parameters for the simulation
            % mdl_puma560;        % Load robot model
            t = (distanceToEndPoint)/velocity;             % Total time (s)
            deltaT = 0.005;      % Control frequency
            steps = round(t/deltaT,0,'decimals');   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector 1s for more weighting than 0.1 angular velocities
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            obj.UR3qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            % Modify the rotation of the end effector based on the object position
            if endPoint(1)>0.01
                objectTransform = troty(-pi/2);
            elseif endPoint(1) <0.01
                objectTransform = troty(pi/2);
            else
                objectTransform = eye(4);
            end
            
            if endPoint(2)>0.01
                objectTransform = objectTransform*trotx(-pi/2);
            elseif endPoint(2) <0.01
                objectTransform = objectTransform*trotx(pi/2);
            end
            
            if facing == 1
                objectTransform = eye(4) * trotx(pi/2) * troty(-pi/2);
            end
            if facing == 2
                objectTransform = eye(4) * troty(pi/2);
            end
            if facing == 3
                objectTransform = eye(4) * trotx(-pi/2) * trotz(pi/2);
            end
            
            rpy = tr2rpy(objectTransform);
            
            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1); % Points in x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Points in y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
                
                theta(1,i) = rpy(1);                                        % Roll angle
                theta(2,i) = rpy(2);                                        % Pitch angle
                theta(3,i) = rpy(3);                                        % Yaw angle
            end
            
            q0 = zeros(1,7);                                                            % Initial guess for joint angles
            obj.UR3qMatrix(1,:) = obj.UR3.model.getpos;                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = obj.UR3.model.fkine(obj.UR3qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(abs(Rd*Ra'));                                            % Convert rotation matrix to RPY angles. Gives error as rotation matrix times its transform should equal the indentity matrix
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = obj.UR3.model.jacob0(obj.UR3qMatrix(i,:));                                          % Get Jacobian at current joint state. Jacob gives with respect to base!!!
                m(i) = sqrt(abs(det(J*J')));
                if m(i) < epsilon                                                       % If manipulability is less than given threshold (epsilon was set at top)
                    lambda = (1 - m(i)/epsilon)*5E-2; % If manipubility is insufficient then damping will be -=< 0
                else
                    lambda = 0; % If manipubility is sufficient then damping will be = 0
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:7                                                             % Loop through joints 1 to 7
                    if obj.UR3qMatrix(i,j) + deltaT*qdot(i,j) < obj.UR3.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif obj.UR3qMatrix(i,j) + deltaT*qdot(i,j) > obj.UR3.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                obj.UR3qMatrix(i+1,:) = obj.UR3qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            Transfrom = obj.UR3.model.fkine(obj.UR3qMatrix(end,:));
            endPosition = Transfrom(1:3,4)';
            
            error = endPosition-endPoint;
            obj.errorMax = max(abs(error));
            
            obj.trajectoryPlot = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);
            
            % Display errors
            disp('End Point Translation Error: ')
            disp(error);
            
            disp('End Point')
            disp(endPosition);
            
        end
        
        function AnimateTrajectory (obj,trajectory,moveTray)
            
            %create the for loop to iterate robots movement throughout the
            %trajectory
            for trajStep = 1:size(trajectory,1)
                Q = trajectory(trajStep,:);
                
                obj.LBR.model.animate(Q);
                obj.LBRGripper1.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2);
                obj.LBRGripper1.model.animate(deg2rad(obj.GripperState));
                obj.LBRGripper2.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2) * troty(pi);
                obj.LBRGripper2.model.animate(deg2rad(obj.GripperState));
                
                if moveTray == true
                    obj.tray.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotz(-pi/2);
                    obj.tray.model.animate(0);
                    obj.bread.model.base = obj.tray.model.fkine(obj.tray.model.getpos()) * trotx(pi/2);
                    obj.bread.model.animate(0);
                    
                end
                drawnow();
            end
        end
        
        function UR3AnimateTrajectory (obj,trajectory,moveBread)
            
            %create the for loop to iterate robots movement throughout the
            %trajectory
            for trajStep = 1:size(trajectory,1)
                
                Q = trajectory(trajStep,:);

                obj.UR3.model.animate(Q);
                
                if moveBread == true
                    obj.bread.model.base = obj.UR3.model.fkine(obj.UR3.model.getpos()) * trotx(pi/2);
                    obj.bread.model.animate(0);
                end
                
                drawnow();
            end
        end

    end
    
end


