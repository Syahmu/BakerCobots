classdef iiwa < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];
        
        %> Flag to indicate if gripper is used
        useGripper = false;
    end
    
    methods%% Class for UR5 robot simulation
        function self = iiwa(useGripper)
            if nargin < 1
                useGripper = false;
            end
            self.useGripper = useGripper;
            
            %> Define the boundaries of the workspace
            
            
            % robot =
            self.GetRobot();
            % robot =
            self.PlotAndColourRobot();%robot,workspace);
        end
        
        %% GetUR5Robot
        % Given a name (optional), create and return a UR5 robot model
        function GetRobot(self)
            %     if nargin < 1
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['IIWA_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %     end
            
            %L1 = Link('d',0,'a',0.34,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            %L2 = Link('d',0,'a',0,'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-120),deg2rad(120)]);
            %L3 = Link('d',0,'a',0.4,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            %L4 = Link('d',0,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-120),deg2rad(120)]);
            %L5 = Link('d',0,'a',0.4,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            %L6 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120),deg2rad(120)]);
            %L7 = Link('d',0,'a',0.126,'alpha',0,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)]);
            
            L(1) = Link([0 0.34 0 -pi/2]); % 0.33997
            L(2) = Link([0 0 0 pi/2]);
            L(3) = Link([0 0.4 0 pi/2]);
            L(4) = Link([0 0 0 -pi/2]);
            L(5) = Link([0 0.4 0 -pi/2]); % 0.39998
            L(6) = Link([0 0 0 pi/2]);
            L(7) = Link([0 0.126 0 0]);
            
            % Incorporate joint limits
            L(1).qlim = [-170 170]*pi/180;
            L(2).qlim = [-120 120]*pi/180;
            L(3).qlim = [-170 170]*pi/180;
            L(4).qlim = [-120 120]*pi/180;
            L(5).qlim = [-170 170]*pi/180;
            L(6).qlim = [-120 120]*pi/180;
            L(7).qlim = [-175 175]*pi/180;
            
            
            self.model = SerialLink(L,'name',name);
            
            self.model.base = self.model.base * transl(0,0,0);
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end