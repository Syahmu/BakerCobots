
RobotGrip(1, transl(0,0,0)*troty(pi/2))


%How to use base model on Matlab code?
%Simlutaneous movement without jumpyness?
%variables to pass into the robotgrip function

%%

function RobotGrip(gripfunct, endeffloc) % gripfunct = 0 (opening) / = 1 (closing)

gripersize = 0.0582 %m

B(1) = Link([pi     0       0.0455       pi/2    1]);

B(2) = Link([pi     0       0       pi/2    0]);

B(1).qlim = [-gripersize/2 0];
B(2).qlim = [-gripersize/2 0];

self.bottom = SerialLink(B,'name','gripper');

T(1) = Link([pi     0       0.0455       pi/2    1]);

T(2) = Link([pi     0       0       pi/2    0]);

T(1).qlim = [gripersize/2 0];
T(2).qlim = [gripersize/2 0];

self.top = SerialLink(T,'name','gripper');

workspace = [-0.2 0.2 -0.2 0.2 -0.1 0.2];  

scale = 0.5;
        
qb = zeros(1,2);  
qt = zeros(1,2); 



self.bottom.base = transl(0,0,0.01005)*endeffloc; %end.eff
self.top.base = self.bottom.base;


self.bottom.plot(qb,'workspace',workspace,'scale',scale);
self.top.plot(qt,'workspace',workspace,'scale',scale);



 
hold on
if gripfunct == 1
    for z = -gripersize/2:0.0004:-0.0003
        newQb = [z,0]
        newQt = [-z,0]
        %hold on
        self.bottom.animate(newQb);
        %drawnow();
        %hold on
        self.top.animate(newQt);
        drawnow();  
    
    end
else gripfunct == 0
    for z = -0.0003:-0.0004:-gripersize/2
    newQb = [z,0]
    newQt = [-z,0]
    %hold on
    self.bottom.animate(newQb);
    self.top.animate(newQt);  
    end
end
end


