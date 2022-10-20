classdef SetScene < handle
    %SETSCENE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        object;
    end
    
    methods
        function obj = SetScene()
            %SETSCENE Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function [mesh_h] = PlaceObject(obj,name, locations)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % Loads object into environment in specified position and orientation
            
            [f,v,data] = plyread(name,'tri');
            
            if nargin < 2
                locations = [0,0,0];
            end
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            try
                try
                    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                catch
                    try
                        vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;
                    catch
                        vertexColours = [0.5,0.5,0.5];
                    end
                end
                
                mesh_h = zeros(size(locations,1),1);
                for i = 1: size(locations,1)
                    % Plot the Trisurf
                    mesh_h(i) = trisurf(f,v(:,1)+locations(i,1),v(:,2)+locations(i,2), v(:,3)+locations(i,3) ...
                        ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
                    
                    % An alternative way of plotting with different edge colours and lighting
                    %             mesh_h(i) = trisurf(f,v(:,1)+locations(i,1),v(:,2)+locations(i,2), v(:,3)+locations(i,3) ...
                    %                 ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                    
                end
                % Catch the case where there are no colours
            catch ME_1
                disp(ME_1);
            end
        end
        
        function [objectArray] = BuildEnvironment(obj,trayPose,tableHeight)
            hold on;
            
            oven = obj.PlaceObject("Oven_brickv1.ply",[0,-1.25,0]);
            table = obj.PlaceObject("BenchTop1.ply",[0,0.5,0]);
            shelf = obj.PlaceObject("ShelfDemos.ply",[0,2,0]);
            
            %add extra ply files
           fence = obj.PlaceObject("barricades.ply",[1.5,0,0]);
           EmergencyButton = obj.PlaceObject("EmergencyButton.ply",[-1,-3,2]);
           ForkLift = obj.PlaceObject("Forklift.ply",[-1,3,0]);
           Worker = obj.PlaceObject("avatar.ply",[0,-3.5,0]);
           Pallets = obj.PlaceObject("pallet.ply",[2,-3.5,0]);
           
            %Add wall and floor Placements
            surf([-1,-1;4,4],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',imread('porcelainFloor.jpg'),'FaceColor','texturemap');
            surf([-1,-1;-1,-1],[-5,5;-5,5],[0.01,0.01;5,5],'CData',imread('wallTile.jpg'),'FaceColor','texturemap');
            xyzlabel;
           
           
       

            objectArray = {oven,table,shelf};
        end
        
    end
end

