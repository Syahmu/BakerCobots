function SetScene(tableHeight,ur3, LBR)
        hold on

        %inside base of oven z = 0.91 (Place trays with breads on top of this
        PlaceObject('Oven_brickv1.ply',[0,-1.5,0]);
        
        
        %Top of table z = 0.79
        PlaceObject('BenchTop1.ply',[0,0.5,0]);
        
        PlaceObject('ShelfDemos.ply',[0,2,0]);
        
        
        %Need to be used outside of this function in order for it to move
        %with robots
        PlaceObject('Baking_sheet_demo.ply',[0,0.5,0.79]);
        PlaceObject('Loaf1.ply',[0.2,0.5,0.79+0.013]);
        PlaceObject('Loaf1.ply',[-0.2,0.5,0.79+0.013]);
        
        ur3.model.base = ur3.model.base * transl(0,1,0.79);
        LBR.model.base = LBR.model.base * transl(0,0,0.79);

        axis equal
        %surf([0,0;3,3],[0,5;0,5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

end