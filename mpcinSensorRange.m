function out = mpcinSensorRange(agent, obstacle , agent_vel)
% inSensorRange - Returns true if the obstalce is in sensor range
%
% Syntax: out = inSensorRange(agent, obstacle)
%
    distance = sum((agent.mpcpos(1:2) - obstacle.mpcpos(1:2)).^2) < agent.sensorRange^2;
    sameSide = (agent.mpcpos(1:2) - obstacle.mpcpos(1:2)) * (agent_vel - obstacle.vel)' < 0;
    if distance == 1 && norm(obstacle.vel) == 0
        sameSide = 1;
    end
    
    out = distance && sameSide;
    
    %
    if sqrt(sum((agent.mpcpos(1:2) - obstacle.mpcpos(1:2)).^2)) <= 6*agent.radius 
        out =1;
    end
    %}
end