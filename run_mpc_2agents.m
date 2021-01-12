%add agent with initial pose , goal and vmax.
%{
agents = [addAgent('1',[3,4,pi/6],[0,0,0],[7,-9,pi/4],1),
          addAgent('2',[-5,0,pi/6],[0,0,0],[5,0,pi/4],1)];
%}
%{
agents = [
    addAgent('1', [-5,-5,pi/4], [0 0 0], [5,5,pi/4],1),
   % addAgent('2', [5 -5,3*pi/4],   [0 0 0], [-5 5,3*pi/4],1),
   % addAgent('3', [-5 5, -pi/4],  [0 0 0], [5 -5, -pi/4],1),
    addAgent('4', [5,5,-3*pi/4],  [0 0 0], [-5 ,-5 ,-3*pi/4],1),
    ];
%}
%{
agents=[
     addAgent('1', [-8 0 0], [0 0 0], [8 0 0],1),
     addAgent('2', [-6 4 atan2(-1,1)],   [0 0 0], [6 -8 atan2(-1,1)],1),
     addAgent('3', [-6 -3 atan2(3,7)],     [0 0 0], [8 3 atan2(3,7)],1),
     addAgent('4', [0 -8 atan2(16,0)],  [0 0 0], [0 8 atan2(16,0)],1),
     addAgent('5', [-3 -6 atan2(13,11)],  [0 0 0], [8 7 atan2(13,11)],1), 
     addAgent('6', [4 -7 atan2(15,-8)],  [0 0 0], [-4 8 atan2(15,-8)],1), 
     addAgent('7', [3 6 atan2(-12,0)],  [0 0 0], [3 -6 atan2(-12,0)],1), 
     addAgent('8', [5 -6 atan2(13,0)],  [0 0 0], [5 7 atan2(13,0)],1), 
     addAgent('9', [-3 6 atan2(-12,0)],  [0 0 0], [-3 -6 atan2(-12,0)],1), 
];
%}
%{
%config2
agents=[
     addAgent('1', [-6 -2 atan2(4,10)], [0 0 0 ], [4 2 atan2(4,10)],1),
     addAgent('2', [-5 3 atan2(-6,9)],   [0 0 0], [4 -3 atan2(-6,9)],1),
     addAgent('3', [-4 -4 atan2(8,6)],     [0 0 0], [2 4 atan2(8,6)],1),
     addAgent('4', [4 0 atan2(0,-12)],  [0 0 0], [-8 0 atan2(0,-12)],1),
     addAgent('5', [4 -2 atan2(4,-11)],  [0 0 0], [-7 2 atan2(4,-11)],1), 
     addAgent('6', [2 -4 atan2(9,-3)],  [0 0 0], [-1 5 atan2(9,-3)],1), 

];
%}
for i = 1:length(agents)
%N - Number of MPC predictions to make
agents(i).N = 10 ;
%M - Frequency of replanning
agents(i).M = 3;
end
%Time Step 
dt = 0.1;
counter = 0;
m = 1;
maxIterations = 500 * agents(1).M;
axisLimits = [-10 10 -10 10];
Vel = [];
Theta = [];
Costs = [];
while counter < maxIterations
 
 for i = 1:length(agents)
    
     if(agents(i).gflag ~= 1) 
        obstacles = [];
        for j = 1:length(agents)
            if i ~= j
                if inSensorRange(agents(i), agents(j))
                    obstacles = [obstacles; agents(j)];
                end
            end
        end
        agents(i).obs = obstacles;
     end
 end
   cost = [];
 for i = 1:length(agents)
%MPC Planner - Holonomic 
agents(i).velocities = comp_mpc(agents,i,dt,counter);
cost = getcosts(agents(i).velocities,agents(i),dt);
Costs = [Costs; cost];
[agents(i).xvels , agents(i).yvels, agents(i).omegas] = getnvels(agents(i));
 end

for j = 1:agents(i).M
    vel = [];
    theta = [];
   for i = 1:length(agents)
       if agents(i).gflag ~= 1
agents(i).path = [agents(i).path ; agents(i).position];
agents(i).position = calc_pos(agents(i),m,dt);
agents(i).vel = [agents(i).xvels(m),agents(i).yvels(m)];
agents(i).w = agents(i).omegas(m);
vel = [vel, agents(i).vel];
theta = [theta,agents(i).position(3)];
agents(i).goaldist = sqrt( sum((agents(i).position - agents(i).goal).^2));
if agents(i).goaldist < 0.5
    agents(i).gflag = 1;
    agents(i).vel = [0,0];
    agents(i).position = agents(i).position;
end
       end
   end
   plotSimulation(agents,counter,dt,axisLimits,true,m);
%Vel = [Vel;vel];
%Theta = [Theta ; theta];

if m < agents(1).M
m = m + 1;
else
    m = 1;
end

end

counter = counter + 1;
sum1 =0;
for k = 1 : length(agents)
if agents(k).gflag == 1
    sum1 = sum1 + 1;
end
end

if sum1 == length(agents)
   break
end

end


