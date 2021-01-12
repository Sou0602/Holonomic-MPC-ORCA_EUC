function vels = comp_mpc(agents,q,dt,counter)
agent = agents(q);
N = agent.N;
cost = @(u)  500* 1/2*getcost(agent,u',dt) + 0.1*1/2.2 * velcost(u',agent) +0.000*1/1.2 *omegacost(u',agent) +0.0001*normalcost(u',agent,dt)+ 0*1/12*sum(u(1:N).^2) +0* 1/12*sum(u(N+1:2*N).^2) + 0*1/100*sum(u(2*N +1 : 3*N).^2);
init = agent.velocities' ;
lb = ones(1,2*N) * -agent.vmax;
lb = [lb , ones(1,N) * -agent.wmax];
ub = ones(1,3*N) * (agent.vmax);

% xvels = agent.velocities(1:N)
% yvels = agent.velocities(N+1:2N)
A = [];
B = [];
Aeq = [];
Beq = [];
cons = [];
constraints = [];
pos_store = [];
vel_store = [];
%vx acceleration

for i = 1 : N - 1
    ax = zeros(1,3*N);
    ax(i+1) = 1;
    ax(i) = -1;
    bx = dt * 1;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(i+1) = -1;
    ax(i) = 1;
    bx = dt * 1.5;
    A = [A;ax];
    B = [B;bx];
end
%vy acceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(N+i+1) = 1;
    ax(N+i) = -1;
    bx = dt * 1;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(N+i+1) = -1;
    ax(N+i) = 1;
    bx = dt * 1.5;
    A = [A;ax];
    B = [B;bx];
end
%angular acceleration 
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(2*N+i+1) = 1;
    ax(2*N+i) = -1;
    bx = dt * 1/5;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(2*N+i+1) = -1;
    ax(2*N+i) = 1;
    bx = dt * 1/5;
    A = [A;ax];
    B = [B;bx];
end

%
if counter ~= 0
%vx,vy,w continuity
vx = agent.vel(1);
vy = agent.vel(2);
w = agent.w;
aeq1 = zeros(1,3*N);
aeq1(1) = 1;
beq1 = vx;
Aeq = [Aeq;aeq1];
Beq = [Beq;beq1];
%{
A = [A;aeq1];
B = [B;beq1];
A = [A;-aeq1];
B = [B;-beq1];
%}
aeq2 = zeros(1,3*N);
aeq2(1 + N) = 1;
beq2 = vy;
Aeq = [Aeq;aeq2];
Beq = [Beq;beq2];
%{
A = [A;aeq2];
B = [B;beq2];
A = [A;-aeq2];
B = [B;-beq2];
%}
aeq3 = zeros(1,3*N);
aeq3(1 + 2*N) = 1;
beq3 = w;
Aeq = [Aeq;aeq3];
Beq = [Beq;beq3];
%{
A = [A;aeq3];
B = [B;beq3];
A = [A;-aeq3];
B = [B;-beq3];
%}
end
%linear approx
%}

%% Linear ORCA

    for k = 1:length(agents)  
        agents(k).mpcpos = agents(k).position(1:2);
    end
%

    for j = 1:length(N)
        if ~isempty(agents(q).obs)
         for x = 1:length(agents(q).obs)
            obstacle = agents(q).obs(x);
            obs_vel = obstacle.vel;
            obs_pos = obstacle.mpcpos(1:2);
            agent_pos = agents(q).mpcpos(1:2);
           % agent_vel0 = [agent.velocities(j) , agent.velocities(N+j)];
           agent_vel0 = agents(q).vel;
           % v = [];
           % nor = [];
            [v,nor] =   getorca_lin(agent,agent_pos,obs_pos,agent_vel0,obs_vel,j,N);
             a = zeros(1,3*agent.N);     
             a(j) = -nor(1);
             a(N+j) = -nor(2);
             b = -(agent_vel0(1)+v(1)*0.5)*nor(1) -(agent_vel0(2)+v(2)*0.5)*nor(2) ;
             A = [A;a];
             B = [B;b];
             pos_store = [pos_store;agent_pos,obs_pos];
             vel_store = [vel_store;j,obs_vel];
          end
        end
        %Position Update
        for k = 1:length(agents)  
         if k ~= q
           agents(k).mpcpos = agents(k).mpcpos + agents(k).vel * dt;
         else
           agents(k).mpcpos = agents(k).mpcpos + agents(k).vel * dt;
           %agents(k).mpcpos = agents(k).mpcpos + [agents(k).velocities(j) agents(k).velocities(N+j)]*dt;
         end
        end
        
        %Obstacle detection
        for m = 1:length(agents)
           obstacles = [];
            if m ~= q
                if mpcinSensorRange(agents(q), agents(m), agents(q).vel)
               % if mpcinSensorRange(agents(q), agents(m), [agents(q).velocities(j) agents(q).velocities(N+j)])
                    obstacles = [obstacles; agents(m)];
                end
            end
            agents(q).obs = obstacles;
        end
        
    end
    constraints = @(u) orca_euc(pos_store,vel_store,N,dt,u,agent.radius);
    %% 
%}
%% Non Linear ORCA/RVO
%{

%constraints = @(u) getorca_nonlin(agent,N,u,dt);
%constraints = @(u) euclidean(agents,u,dt,q);
constraints =  @(u) rvo_hol(agents,u,dt,q); 
%constraints = @(u) euclidean_prev(agent,u,dt);
%}


    options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point');
 
    [vels,~,eflag,~] = fmincon(cost, init, A, B, Aeq, Beq, lb, ub, constraints, options);
   % vels = fmincon(cost, init, A, B, [], [], lb, ub, [], options)';
     vels = vels';
     disp(eflag);
    vels = smoothvels(vels,agent);
end