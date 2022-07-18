%close all; clear ; clc

dt = 0.001; % time step
PopSize = 25;      
MaxGenerations = 10;
s = tf('s');
Rcontr = tf(zeros(circuitNumber,circuitNumber,1));

for i=1:12
    G = sys(i,i)*PSsys;
    
    % optimal problem
    options = optimoptions(@ga,'PopulationSize',PopSize, ...
        'MaxGenerations',MaxGenerations);
    [x,fval] = ga(@(K)pidtest(G,dt,K),3,-eye(3),zeros(3,1))

    Rcontr(i,i) = pid(x(1),x(2),x(3),.001);
end

Controller = struct('R1',pid(Rcontr(1,1)),'R2',pid(Rcontr(2,2)),'R3',pid(Rcontr(3,3)),'R4',pid(Rcontr(4,4)),'R5', ...
    pid(Rcontr(5,5)),'R6',pid(Rcontr(6,6)),'R7',pid(Rcontr(7,7)),'R8',pid(Rcontr(8,8)),'R9',pid(Rcontr(9,9)),'R10', ...
    pid(Rcontr(10,10)),'R11',pid(Rcontr(11,11)),'R12',pid(Rcontr(12,12)));

% cost function
function J = pidtest(G,dt,parms)
    s = tf('s');
    K = parms(1) + parms(2)/s + parms(3)*s/(1+.001*s);
    Loop = series(K,G);
    ClosedLoop = feedback(Loop,1);
    t = 0:dt:0.5;
    [y,t] = step(ClosedLoop,t);
    CTRLtf = K/(1+K*G);
    u = lsim(CTRLtf,1-y,t);
    Q = 1;
    W1 = 0.5;
    J = dt*sum(Q*(1-y(:)).^2+W1*u(:).^2)
    [y,t] = step(ClosedLoop,t);
    plot(t,y,'LineWidth',2,'color','r')
    drawnow
end