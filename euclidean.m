function [c,ceq] = euclidean(agents,u,dt,q)


c = [];
ceq = [];
N = agents(q).N;

for j = 1:N
    vx = u(j);
    vy = u(N+j);
    agent_vel = [vx , vy];
    
 if ~isempty(agents(q).obs)
 for i = 1:length(agents(q).obs)
    
    obstacle = agents(q).obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.mpcpos(1:2);
    agent_pos = agents(q).mpcpos(1:2);
    opos = obs_pos + obs_vel * dt;
    
    %{
          if j ==1
              c(end+1) = u(j) - agent.vel(1);
              c(end+1) = u(j+N) - agent.vel(2); 
              c(end+1) = -1*(u(j) - agent.vel(1));
              c(end+1) = -1*(u(j+N) - agent.vel(2));
          end
    %}
        
           c(end+1)=-1 * (((agent_pos(1) + u(j)*dt - opos(1))^2 + (agent_pos(2) + u(N+j)*dt - opos(2))^2) - 9*agents(q).radius^2);
         
           
 end
 else 
     c(end+1) = -1000;
 end
 % Position Update
 for k = 1:length(agents)  
         if k ~= q
           agents(k).mpcpos = agents(k).mpcpos + agents(k).vel * dt;
         else
           agents(k).mpcpos = agents(k).mpcpos + agent_vel * dt;
         end
 end
 % Obsacle Detection
 for m = 1:length(agents)
           obstacles = [];
            if m ~= q
                if mpcinSensorRange(agents(q), agents(m), agent_vel)
                    obstacles = [obstacles; agents(m)];
                end
            end
            agents(q).obs = obstacles;
 end
               
end

end
        