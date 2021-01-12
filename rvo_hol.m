function [c,ceq] = rvo_hol(agents,u,dt,q)


c = [];
ceq = [];
N = agents(q).N;
agent_v0 = agents(q).vel;

for j = 1:N
    
    
    vrvo = [u(j) u(N+j)];
    
 if ~isempty(agents(q).obs)
 for i = 1:length(agents(q).obs)
    
    obstacle = agents(q).obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.mpcpos(1:2);
    agent_pos = agents(q).mpcpos(1:2);
       
        rij = (agent_pos - obs_pos) ; 
        vdiff = (2*vrvo - agent_v0 - obs_vel);
        
        rij2 = sum(rij.^2);
        vdiff2 = sum(vdiff.^2);
        Rij2 = agents(q).radius + obstacle.radius;
        f = ( vdiff2 * (rij2 - Rij2) ) - ( vdiff * rij' )^2;
        
           c(end+1)=-1*(( vdiff2 * (rij2 - Rij2) ) - ( vdiff * rij' )^2 );
 
 end
 else
     c(end+1) = -100;
 end
 %Position Update
    
        for k = 1:length(agents)  
         if k ~= q
           agents(k).mpcpos = agents(k).mpcpos + agents(k).vel * dt;
         else
        if j < N
        agents(k).mpcpos = agents(k).mpcpos + agent_v0 * dt;
        end
         end
        end
        
  %Obstacle Detecction
  
        for m = 1:length(agents)
           obstacles = [];
            if m ~= q
                if mpcinSensorRange(agents(q), agents(m), agent_v0)
                    obstacles = [obstacles; agents(m)];
                end
            end
            agents(q).obs = obstacles;
        end
        agent_v0 = [u(j) u(N+j)];
end
end