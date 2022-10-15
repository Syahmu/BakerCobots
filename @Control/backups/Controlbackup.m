classdef Control < handle
    %CONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        LBR;
        oven;
        table;
        shelf;
        tray;
        trayPose;
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
    end
    
    methods
        
        function obj = Control(LBR,objectArray, trayPose, avoidCollisions, resolveMotionRateControl)
            %CONTROL Construct an instance of this class
            %   Detailed explanation goes here
            obj.LBR = LBR;
            obj.oven = objectArray{1};
            obj.table = objectArray{2};
            obj.shelf = objectArray{3};
            obj.tray = objectArray{4};
            obj.trayPose = trayPose;
            obj.collisionObjects = obj.CollisionObjects();
            obj.avoidCollisions = avoidCollisions;
            obj.resolveMotionRateControl = resolveMotionRateControl;
            obj.trajectorySteps = 100;
            obj.errorMax = 1;
            obj.moveVelocity = 1;
        end
        
        function MoveTray(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.Start();
        end
        
        function Start(obj)
            point = obj.trayPose.position;
            
            launching = false;
            
            obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity,launching);
        end
        
        function MoveEndEffectorToPoint(obj,point,moveObject,velocity,launching)
            obj.ResolveMotionRateControl(point,moveObject,velocity,launching);
        end
        
        function ResolveMotionRateControl(obj,endPoint,moveObject,velocity,launching)
            % Calculate RMRC trajectory
            obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,launching);
            
            %toggle if avoidcollisions is true
            if obj.avoidCollisions==true
                % Avoid collisions to reach the end point
                %obj.CollisionAvoidance(moveObject,launching);
            end
            
            obj.AnimateTrajectory(obj.qMatrix,moveObject);
            
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
        end
        
        function ResolveMotionRateControlCalculateTrajectory(obj,endPoint,velocity,launching)
            
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
                if launching == true
                    x(3,i) = startPoint(3) + sqrt((hypot(startPoint(2),startPoint(1)))^2-((hypot(x(1,i),x(2,i))))^2);% Points in z
                elseif launching == false
                    x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
                    %     x(3,i) = endPoint(3) + 0.2*sin(i*delta); % Points in z
                    
                end
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
            
            disp('End Point Maximum Translation Error: ')
            disp(obj.errorMax);
            
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
                
                drawnow();
            end
        end
        
        
        %% Collision avoidance
        
        
        function CollisionAvoidance(obj,moveObject,launching)
            
            % Number of collision objects created
            numberOfObjects = size(obj.collisionObjects,2);
            
            % Initialising results container
            results = zeros(1,numberOfObjects);
            
            % End joint config
            obj.endQ = obj.qMatrix(end,:);
            q = obj.endQ;
            
            while true  % Loop until possible to move to end q without collisions
                
                %                 if obj.resolveMotionRateControl == false
                
                if q ~= obj.endQ | obj.resolveMotionRateControl == false
                    % Finding the robot joint positions required to move the end effector to the end point
                    trajectory = jtraj(obj.LBR.model.getpos(),q,obj.trajectoryLength);
                elseif q == obj.endQ & obj.resolveMotionRateControl == true
                    
                    velocity = obj.moveVelocity;
                    endFkine = obj.LBR.model.fkine(q);
                    endPoint = endFkine(1:3,4)';
                    obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,launching);
                    trajectory = obj.qMatrix();
                end
                %                 elseif obj.resolveMotionRateControl == true
                %
                %                     % Move to endPoint using RMRC
                %                     transform = obj.LBR.model.fkine(q);
                %                     endPoint = transform(1:3,4)';
                %                     obj.ResolveMotionRateControlCalculateTrajectory(endPoint,obj.moveVelocity);
                %
                %                 end
                
                delete(obj.collisionPlot);
                obj.collisionPlot = [];
                
                % Check for trajectory collision before animating for each obstacle
                for i=1:1:numberOfObjects
                    
                    obstacle = obj.collisionObjects{i};
                    
                    results(i) = obj.CollisionDetection(trajectory,obstacle{1},obstacle{2},obstacle{3});
                    
                end
                
                % If collision is along trajectory calculate a random new trajectory
                if ismember(1,results)
                    
                    disp('Collision along trajectory: recalculating alternate trajectory');
                    
                    % Calculate a random joint config and its end effector endPoint
                    [q,endPoint] = obj.GenerateRandomQAndPoint();
                else
                    
                    % Move to random pose that is not in collision. Set q to equal the goal end Q and check if it is now in collision
                    if q ~= obj.endQ
                        
                        disp('Moving to random pose that is not in collision');
                        
                        obj.AnimateTrajectory(trajectory,moveObject);
                        
                        q = obj.endQ;
                        
                    else % if there is no collisions to move toward the final q pose
                        
                        disp('No collisions along trajectory: moving to target pose');
                        
                        %                         endQ = obj.endQ;
                        
                        break;
                    end
                    
                end
                
                % Clear result collision checker
                results = [];
                
            end
            
        end
        
        function [collisionObjects] = CollisionObjects(obj)
            
            % Define collision objects
            
            % Floor
            
            centerpnt = [0,0,-2.51];
            %             side = 6;
            height = 5;
            plotOptions.plotFaces = true;
            colour = false;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            benchSurface = {vertex,faces,faceNormals,patch};
            
            % Obstacle box
            
            centerpnt = [0,-0.9,1.2];
            height = 0.5;
            plotOptions.plotFaces = true;
            colour = true;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            door = {vertex,faces,faceNormals,patch};
            
            collisionObjects = {benchSurface,door};
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

