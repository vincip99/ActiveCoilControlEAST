%% sensitivity analisys

Msample = 8;   % number of sample     
Mpercent = 50;  % uncertanty
precision = Mpercent/Msample;   % precision of sampling

% sampling L matrix
Lu = zeros([size(L), 2*Msample+1]); % L uncertain matrix
for i = 1:Msample
    Lu(:,:,i) = L - i .* precision/100 .* L;
end
Lu(:,:,Msample+1) = L;
for i = Msample+2:2*Msample+1
    Lu(:,:,i) = L + i .* precision/100 .* L; 
end

% sampling R matrix
Ru = zeros([size(R), 2*Msample+1]); % R uncertain matrix
for i = 1:Msample
    Ru(:,:,i) = R - i .* precision/100 .* R;
end
Ru(:,:,Msample+1) = R;
for i = Msample+2:2*Msample+1
    Ru(:,:,i) = R + i .* precision/100 .* R; 
end

% system matices for uncertain state-space mimo model
Au = zeros([size(A), 2*Msample+1]); Bu = zeros([size(B), 2*Msample+1]);
Cu = zeros([size(C), 2*Msample+1]); Du = zeros([size(D), 2*Msample+1]);
sysu = ss(zeros([size(sys), 2*Msample+1]));
% sampling state space matrix
for i=1:2*Msample+1
    Au(:,:,i) = -inv(Lu(:,:,i))*Ru(:,:,i);                                                                                    
    Bu(:,:,i) = Lu(:,:,i)\S;                                                
    Cu(:,:,i) = eye(circuitNumber,length(Au(:,:,i)));                      
    Du(:,:,i) = zeros(circuitNumber,circuitNumber); 
    sysu(:,:,i) = ss(Au(:,:,i),Bu(:,:,i),Cu(:,:,i),Du(:,:,i));
end

% matrix of controllers
Rcontr = tf(zeros(circuitNumber,circuitNumber,1));
Rcontr(1,1) = R1; Rcontr(2,2) = R2; Rcontr(3,3) = R3; Rcontr(4,4) = R4; Rcontr(5,5) = R5; Rcontr(6,6) = R6; 
Rcontr(7,7) = R7; Rcontr(8,8) = R8; Rcontr(9,9) = R9; Rcontr(10,10) = R10; Rcontr(11,11) = R11; Rcontr(12,12) = R12;

% open loop uncertanin 
Oloop = tf(zeros([circuitNumber, circuitNumber, 2*Msample+1]));
for i=1:2*Msample+1
    Oloop(:,:,i) = Rcontr * eye(circuitNumber, circuitNumber) * PSsys * sysu(:,:,i);
end


