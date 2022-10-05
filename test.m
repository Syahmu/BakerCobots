robot = Fanuc;

q = [0,0,0,0,0,0];

scale = 0.5;
workspace = [-0.5 1.5 -0.5 1.5 -1 1];                                      
robot.plot(q,'workspace',workspace,'scale',scale); 