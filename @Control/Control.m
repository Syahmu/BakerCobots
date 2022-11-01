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
        resolveMotionRateControl;
        errorMax;
        objectPlot;
        objectPosition;
        qMatrix;
        endQ;
        collisionObjects;
        CollisionDetection;
        GripperState;
        LBRGripper1;
        LBRGripper2;
    end
    
    methods
        
        function obj = Control(LBR,GripperState,LBRGripper1,LBRGripper2,ovenDoor,objectArray, avoidCollisions, resolveMotionRateControl)
            %CONTROL Construct an instance of this class
            %   Detailed explanation goes here
            obj.LBR = LBR;
            obj.GripperState = GripperState;
            obj.LBRGripper1 = LBRGripper1;
            obj.LBRGripper2 = LBRGripper2;
            obj.ovenDoor = ovenDoor;
            obj.oven = objectArray{1};
            obj.table = objectArray{2};
            obj.shelf = objectArray{3};
            obj.collisionObjects = obj.CollisionObjects();  %not working
            obj.avoidCollisions = avoidCollisions;
            obj.resolveMotionRateControl = resolveMotionRateControl;
            obj.trajectorySteps = 100;
            obj.errorMax = 1;
            obj.moveVelocity = 1;
        end
        
        function Start(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %obj.MoveToHandle();
            obj.OpenDoor();
        end
        
        function MoveToHandle(obj)
            handleFkine = obj.ovenDoor.model.fkine(obj.ovenDoor.model.getpos());
            handlePosition = [handleFkine(1,4),handleFkine(2,4),handleFkine(3,4)];
            
            point = handlePosition;
            
            pStar = [500;500] %target point is camera centre (pixel coordinates)
            initQ = obj.LBR.model.getpos() %Initial Q starting position
            %which i have changed to [pi/2,0,0,0,0,-pi/2,0] in the main function for camera roughly facing oven
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ... %perspective Camera (CentralCamera)
            'resolution', [1024 1024], 'centre', [512 512], 'fps', 25, 'name', 'mycamera');
            % with focal length of 0.08, 
            % pixel size of 10e‐5, 
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
            
            obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity);
        end
        
        function OpenDoor(obj)
            for i = 0:-0.025:-0.7
                obj.ovenDoor.model.animate(i);
                handleFkine = obj.ovenDoor.model.fkine(obj.ovenDoor.model.getpos());
                handlePosition = [handleFkine(1,4),handleFkine(2,4),handleFkine(3,4)];
                obj.MoveEndEffectorToPoint(handlePosition,false,obj.moveVelocity);
            end
        end
        
        
        function MoveEndEffectorToPoint(obj,point,moveObject,velocity)
            obj.ResolveMotionRateControl(point,moveObject,velocity);
        end
        
        function ResolveMotionRateControl(obj,endPoint,moveObject,velocity)
            % Calculate RMRC trajectory
            obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity);
            
            %toggle if avoidcollisions is true
            if obj.avoidCollisions==true
                % Avoid collisions to reach the end point
                %obj.CollisionAvoidance(moveObject,launching);
            end
            
            obj.AnimateTrajectory(obj.qMatrix,moveObject);
            
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
        end
        
        function ResolveMotionRateControlCalculateTrajectory(obj,endPoint,velocity)
            
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
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
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
            
            rpy = tr2rpy(objectTransform);
            
            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1); % Points in x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Points in y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
                
                test = 1;
                if velocity>3
                    test = -1;
                end
                theta(1,i) = test*rpy(1);                                        % Roll angle
                theta(2,i) = test*rpy(2);                                        % Pitch angle
                theta(3,i) = test*rpy(3);                                        % Yaw angle
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
                    lambda = (1 - m(i)/epsilon)*5E-2; % If manipubility is insufficient then damping will be -=< 0
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
        
        function AnimateTrajectory (obj,trajectory,moveObject)
            
            % Iterate the robot arms through their movement
            for trajStep = 1:size(trajectory,1)
                
                Q = trajectory(trajStep,:);
                
                % calculate end effector position using fkine
                fkine = obj.LBR.model.fkine(obj.LBR.model.getpos());
                endEffectorPosition = fkine(1:3,4);
                
                if moveObject == true
                    % Move object to the end effector position for each
                    % trajectory step (simulates ball movement)
                    obj.MoveObject(obj.object,endEffectorPosition,0);
                end
                
                % Animate robot through a fraction of the total movement
                obj.LBR.model.animate(Q);
                obj.LBRGripper1.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2);
                obj.LBRGripper1.model.animate(deg2rad(obj.GripperState));
                obj.LBRGripper2.model.base = obj.LBR.model.fkine(obj.LBR.model.getpos()) * trotx(pi/2) * troty(pi);
                obj.LBRGripper2.model.animate(deg2rad(obj.GripperState));
                
                
                drawnow();
            end
        end
        
        
        %% Collision avoidance
        function [collisionObjects] = CollisionObjects(obj)
            % Oven Hitbox
            centerpnt = [0,-1.3,1.2];
            height = 0.5;
            plotOptions.plotFaces = true;
            colour = true;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            door = {vertex,faces,faceNormals,patch};
            
            collisionObjects = {door};
        end
        
        function [vertex,face,faceNormals,collisionObject] = RectangularPrism(obj,lower,upper,plotOptions,colour,axis_h)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % From 41013 Robotics week 5 material
            % "RectangularPrism.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if nargin<4
                axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=true;
                    plotOptions.plotFaces=true;
                end
            end
            hold on
            
            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;
            
            face=[1,2,3;1,3,7;
                1,6,5;1,7,5;
                1,6,4;1,4,2;
                6,4,8;6,5,8;
                2,4,8;2,3,8;
                3,7,5;3,8,5;
                6,5,8;6,4,8];
            
            if 2 < nargout
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            % If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1);
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end
            
            % If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];
                
                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end
            
            % If you want to plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                
                tcolor = [.2 .2 .8];
                
                if colour == true
                    
                    transparency = 1; % 1 = 100% opaque
                    
                else
                    
                    transparency = 0; % 0 = 100% transparent
                    
                end
                
                collisionObject = patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','FaceAlpha',transparency,'lineStyle','none');
            end
        end
        
    end
end

