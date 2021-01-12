function [c,ceq] = orca_euc(pos_store,vel_store,N,dt,u,r)
c = [];
ceq = [];

for i = 1:size(pos_store,1)
    
    agent_pos = pos_store(i,1:2);
    obs_pos = pos_store(i,3:4);
    j = vel_store(i,1);
    obs_vel = vel_store(i,2:3);

    opos = obs_pos + obs_vel * dt;



c(end+1) = - (((agent_pos(1) + u(j)*dt - opos(1))^2 + (agent_pos(2) + u(N+j)*dt - opos(2))^2) - 15*r^2);
end

end