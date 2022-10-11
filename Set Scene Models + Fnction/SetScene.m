function SetScene(Options)
Options = 1


switch(Options)

    case 1
        
        workspace = [-5 5 -5 5 -5 5]
         hold on
        
         %% Place UR3 Robot in Place
         
%          ur3Robot = LinearUR3b(false);
%          ur3Robot.model.base = transl(0.5,1.8,0.79)
%          ur3Robot.animate(0);
         
        
        
         %inside base of oven z = 0.91 (Place trays with breads on top of this
        PlaceObject('Oven_brickv1.ply',[0,0.5,0])
         
         
         %Top of table z = 0.79
         PlaceObject('BenchTop1.ply',[0.2,1.6,0])
         
         PlaceObject('ShelfDemos.ply',[0.5,4,0])
         
         
         %Need to be used outside of this function in order for it to move
         %with robots
         PlaceObject('Baking_sheet_demo.ply',[0.2,1.9,0.8])
         PlaceObject('Loaf1.ply',[0.5,2,0.85])
         PlaceObject('Loaf1.ply',[0.8,2,0.85])
         
         
         
         surf([0,0;3,3],[0,5;0,5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
        hold on
        
        
        
        
     axis equal
        
    case 2

end











end