function cost = normalcost(u,agent,dt)
N = agent.N;
normdis = zeros(N,1);
normpoints = zeros(N,2);

ix = agent.initialpos(1);
iy = agent.initialpos(2);
    
    a = agent.goal(1) - ix;
    b = agent.goal(2) - iy;
    c = sqrt(a^2 + b^2);
    a = a/c;
    b = b/c;
px = agent.position(1);
py = agent.position(2);
for i = 1:N
    px = px + u(i) * dt;
    py = py + u(N+i) * dt;
    ny = (a*b*(px - ix) + b^2 * py + a^2 * iy )/ (a^2 + b^2);
    nx = px - (b/a) * (ny - py);
    normpoints(i,:) = [nx,ny];
    
    normdis(i) = (nx - px)^2 + (ny - py)^2;
end
   cost = sum(normdis); 
end
