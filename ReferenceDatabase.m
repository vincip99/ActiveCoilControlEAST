%% 
close all; clear; clc;

%% signal 1
load ('Dati\107845_magdata.mat','pf','t')
           
%% transformation matrix
KK=[ 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF1
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF2
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0  %PF3
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0  %PF4
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0  %PF5
     0 0 0 0 0 1  0 0 0  0 0 0 0 0 0  %PF6
     0 0 0 0 0 0 .5 0  .5 0  0 0 0 0 0 %PF_7_9
     0 0 0 0 0 0  0 .5  0 .5 0 0 0 0 0 %PF_8_10
     0 0 0 0 0 0  0  0  0  0 1 0 0 0 0 %PF11
     0 0 0 0 0 0  0  0  0  0 0 1 0 0 0 %PF12
     0 0 0 0 0 0  0  0  0  0 0 0 1 0 0 %PF13
     0 0 0 0 0 0  0  0  0  0 0 0 0 1 0 %
     % 0 0 0 0 0 0  0  0  0  0 0 0 0 0 1 %IC
     ];
 
ipf=KK*pf';

%% Reference 
%equal timestep
dt=1e-3;
tdec=t(1):dt:t(end);
Ipf=interp1(t,ipf',tdec);

%% signal build
tsig1 = [0, 2, 3, 5];

varIpf1 = var(Ipf(3001 : 4001,:));
steadycurr(1:6) = 2000;
steadycurr(7:10) = 1800;
steadycurr(11:12) = 2000;
coeff1 = (steadycurr / (tsig1(2) -tsig1(1)))';
coeff2 = (-steadycurr / (tsig1(4) -tsig1(3)))';

unitstep0 = tdec>=tsig1(1);
unitstep1 = tdec>=tsig1(2);
unitstep2 = tdec>=tsig1(3);
unitstep3 = tdec>=tsig1(4);
ramp0 = (tdec-tsig1(1)).*unitstep0;
ramp1 = (tdec-tsig1(2)).*unitstep1;
ramp2 = (tdec-tsig1(3)).*unitstep2;
ramp3 = (tdec-tsig1(4)).*unitstep3;

rif1 = coeff1.*ramp0-coeff1.*ramp1+coeff2.*ramp2-coeff2.*ramp3;
simt1 = tdec';
simrif1 = rif1';
simIn1 = [simt1 simrif1];

Reference1 = struct('signal',simrif1,'time',simt1,'simulink',simIn1,'var',varIpf1);


%% signal 2
load('Dati\100740_magdata.mat','pf','t')
    
%%  transformation matrix
KK=[ 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF1
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF2
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0  %PF3
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0  %PF4
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0  %PF5
     0 0 0 0 0 1  0 0 0  0 0 0 0 0 0  %PF6
     0 0 0 0 0 0 .5 0  .5 0  0 0 0 0 0 %PF_7_9
     0 0 0 0 0 0  0 .5  0 .5 0 0 0 0 0 %PF_8_10
     0 0 0 0 0 0  0  0  0  0 1 0 0 0 0 %PF11
     0 0 0 0 0 0  0  0  0  0 0 1 0 0 0 %PF12
     0 0 0 0 0 0  0  0  0  0 0 0 1 0 0 %PF13
     0 0 0 0 0 0  0  0  0  0 0 0 0 1 0 %
     % 0 0 0 0 0 0  0  0  0  0 0 0 0 0 1 %IC
     ];
 
ipf=KK*pf';

%% reference
%equal timestep
dt=1e-3;
tdec=t(1):dt:t(end);
Ipf=interp1(t,ipf',tdec);

%% signal build
tsig2 = (0:2:23)';
tsigtmp = (1:2:24)';

varIpf2 = var(Ipf(1001 : 2001,:));
steadycurr(1:6) = 250;
steadycurr(7:8) = 150;
steadycurr(9:10) = 300;
steadycurr(11:12) = 250;

unitstep = tdec>=tsig2 & tdec<=tsigtmp;

rif2 = zeros(size(unitstep));
for i=1:length(tsig2)-1
    rif2(i,:) = unitstep(i,:)*steadycurr(i);
end
simt2 = tdec';
simrif2 = rif2';
simIn2 = [simt2 simrif2];

Reference2 = struct('signal',simrif2,'time',simt2,'simulink',simIn2,'var',varIpf2);


%% signal 3
load ('Dati\107846_magdata.mat','pf','t')
        
%% transformation matrix
KK=[ 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF1
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF2
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0  %PF3
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0  %PF4
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0  %PF5
     0 0 0 0 0 1  0 0 0  0 0 0 0 0 0  %PF6
     0 0 0 0 0 0 .5 0  .5 0  0 0 0 0 0 %PF_7_9
     0 0 0 0 0 0  0 .5  0 .5 0 0 0 0 0 %PF_8_10
     0 0 0 0 0 0  0  0  0  0 1 0 0 0 0 %PF11
     0 0 0 0 0 0  0  0  0  0 0 1 0 0 0 %PF12
     0 0 0 0 0 0  0  0  0  0 0 0 1 0 0 %PF13
     0 0 0 0 0 0  0  0  0  0 0 0 0 1 0 %
     % 0 0 0 0 0 0  0  0  0  0 0 0 0 0 1 %IC
     ];
 
ipf=KK*pf';

%% reference 
%equal timestep
dt=1e-3;
tdec=t(1):dt:t(end);
Ipf=interp1(t,ipf',tdec);

%% signal build
tsig3 = [0, 1, 3, 4, 5, 7, 8];
tsig3offset = 10;

varIpf3 = var(Ipf(3001 : 4001,:));
steadycurr(1:6) = 4000;
steadycurr(7:12) = 0;

coeff1 = (steadycurr / (tsig3(2) - tsig3(1)))';
coeff2 = (-steadycurr / (tsig3(4) - tsig3(3)))';
coeff3 = coeff2;
coeff4 = coeff1;

unitstep0 = zeros(6,length(tdec));
unitstep1 = zeros(6,length(tdec));
unitstep2 = zeros(6,length(tdec));
unitstep3 = zeros(6,length(tdec));
unitstep4 = zeros(6,length(tdec));
unitstep5 = zeros(6,length(tdec));
unitstep6 = zeros(6,length(tdec));
ramp0 = zeros(6,length(tdec));
ramp1 = zeros(6,length(tdec));
ramp2 = zeros(6,length(tdec));
ramp3 = zeros(6,length(tdec));
ramp4 = zeros(6,length(tdec));
ramp5 = zeros(6,length(tdec));
ramp6 = zeros(6,length(tdec));
ramp7 = zeros(6,length(tdec));

for i=1:6
    unitstep0(i,:) = tdec>=(tsig3(1)+(i-1)*tsig3offset);
    unitstep1(i,:) = tdec>=(tsig3(2)+(i-1)*tsig3offset);
    unitstep2(i,:) = tdec>=(tsig3(3)+(i-1)*tsig3offset);
    unitstep3(i,:) = tdec>=(tsig3(4)+(i-1)*tsig3offset);
    unitstep4(i,:) = tdec>=(tsig3(5)+(i-1)*tsig3offset);
    unitstep5(i,:) = tdec>=(tsig3(6)+(i-1)*tsig3offset);
    unitstep6(i,:) = tdec>=(tsig3(7)+(i-1)*tsig3offset);
    ramp0(i,:) = (tdec-tsig3(1)-(i-1)*tsig3offset).*unitstep0(i,:);
    ramp1(i,:) = (tdec-tsig3(2)-(i-1)*tsig3offset).*unitstep1(i,:);
    ramp2(i,:) = (tdec-tsig3(3)-(i-1)*tsig3offset).*unitstep2(i,:);
    ramp3(i,:) = (tdec-tsig3(4)-(i-1)*tsig3offset).*unitstep3(i,:);
    ramp4(i,:) = ramp3(i,:);
    ramp5(i,:) = (tdec-tsig3(5)-(i-1)*tsig3offset).*unitstep4(i,:);
    ramp6(i,:) = (tdec-tsig3(6)-(i-1)*tsig3offset).*unitstep5(i,:);
    ramp7(i,:) = (tdec-tsig3(7)-(i-1)*tsig3offset).*unitstep6(i,:);
end

rif3 = zeros(length(steadycurr),length(tdec));
for i=1:6
    rif3(i,:) = coeff1(i).*ramp0(i,:)-coeff1(i).*ramp1(i,:)+coeff2(i).*ramp2(i,:)-coeff2(i).*ramp3(i,:) ... 
    + (coeff3(i).*ramp4(i,:)-coeff3(i).*ramp5(i,:)+coeff4(i).*ramp6(i,:)-coeff4(i).*ramp7(i,:));
end

simt3 = tdec';
simrif3 = rif3';
simIn3 = [simt3 simrif3];

Reference3 = struct('signal',simrif3,'time',simt3,'simulink',simIn3,'var',varIpf3);


%% signal 4
load('Dati\107847_magdata.mat','pf','t')
        
%% transformation matrix
KK=[ 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF1
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0  %PF2
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0  %PF3
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0  %PF4
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0  %PF5
     0 0 0 0 0 1  0 0 0  0 0 0 0 0 0  %PF6
     0 0 0 0 0 0 .5 0  .5 0  0 0 0 0 0 %PF_7_9
     0 0 0 0 0 0  0 .5  0 .5 0 0 0 0 0 %PF_8_10
     0 0 0 0 0 0  0  0  0  0 1 0 0 0 0 %PF11
     0 0 0 0 0 0  0  0  0  0 0 1 0 0 0 %PF12
     0 0 0 0 0 0  0  0  0  0 0 0 1 0 0 %PF13
     0 0 0 0 0 0  0  0  0  0 0 0 0 1 0 %
     % 0 0 0 0 0 0  0  0  0  0 0 0 0 0 1 %IC
     ];
 
ipf=KK*pf';

%% reference
%equal timestep
dt=1e-3;
tdec=t(1):dt:t(end);
Ipf=interp1(t,ipf',tdec);

%% signal build
tsig4 = [0, 1, 3, 4, 5, 7, 8];
tsig4offset = 10;

varIpf4 = var(Ipf(3001 : 4001,:));
steadycurr(1:6) = 0;
steadycurr(7:8) = 2000;
steadycurr(9:10) = 3000;
steadycurr(11:12) = 4000;

coeff1 = (steadycurr / (tsig4(2) - tsig4(1)))';
coeff2 = (-steadycurr / (tsig4(4) - tsig4(3)))';
coeff3 = coeff2;
coeff4 = coeff1;

unitstep0 = zeros(6,length(tdec));
unitstep1 = zeros(6,length(tdec));
unitstep2 = zeros(6,length(tdec));
unitstep3 = zeros(6,length(tdec));
unitstep4 = zeros(6,length(tdec));
unitstep5 = zeros(6,length(tdec));
unitstep6 = zeros(6,length(tdec));
ramp0 = zeros(6,length(tdec));
ramp1 = zeros(6,length(tdec));
ramp2 = zeros(6,length(tdec));
ramp3 = zeros(6,length(tdec));
ramp4 = zeros(6,length(tdec));
ramp5 = zeros(6,length(tdec));
ramp6 = zeros(6,length(tdec));
ramp7 = zeros(6,length(tdec));

for i=1:6
    unitstep0(i,:) = tdec>=(tsig4(1)+(i-1)*tsig4offset);
    unitstep1(i,:) = tdec>=(tsig4(2)+(i-1)*tsig4offset);
    unitstep2(i,:) = tdec>=(tsig4(3)+(i-1)*tsig4offset);
    unitstep3(i,:) = tdec>=(tsig4(4)+(i-1)*tsig4offset);
    unitstep4(i,:) = tdec>=(tsig4(5)+(i-1)*tsig4offset);
    unitstep5(i,:) = tdec>=(tsig4(6)+(i-1)*tsig4offset);
    unitstep6(i,:) = tdec>=(tsig4(7)+(i-1)*tsig4offset);
    ramp0(i,:) = (tdec-tsig4(1)-(i-1)*tsig4offset).*unitstep0(i,:);
    ramp1(i,:) = (tdec-tsig4(2)-(i-1)*tsig4offset).*unitstep1(i,:);
    ramp2(i,:) = (tdec-tsig4(3)-(i-1)*tsig4offset).*unitstep2(i,:);
    ramp3(i,:) = (tdec-tsig4(4)-(i-1)*tsig4offset).*unitstep3(i,:);
    ramp4(i,:) = ramp3(i,:);
    ramp5(i,:) = (tdec-tsig4(5)-(i-1)*tsig4offset).*unitstep4(i,:);
    ramp6(i,:) = (tdec-tsig4(6)-(i-1)*tsig4offset).*unitstep5(i,:);
    ramp7(i,:) = (tdec-tsig4(7)-(i-1)*tsig4offset).*unitstep6(i,:);
end

rif4 = zeros(length(steadycurr),length(tdec));
for i=1:6
    rif4(i+6,:) = coeff1(i+6).*ramp0(i,:)-coeff1(i+6).*ramp1(i,:)+coeff2(i+6).*ramp2(i,:)-coeff2(i+6).*ramp3(i,:) ... 
    + (coeff3(i+6).*ramp4(i,:)-coeff3(i+6).*ramp5(i,:)+coeff4(i+6).*ramp6(i,:)-coeff4(i+6).*ramp7(i,:));
end

simt4 = tdec';
simrif4 = rif4';
simIn4 = [simt4 simrif4];

Reference4 = struct('signal',simrif4,'time',simt4,'simulink',simIn4,'var',varIpf4);

Scenario = struct('Reference1',Reference1,'Reference2',Reference2,'Reference3',Reference3,'Reference4',Reference4);
save('Scenario.mat','Scenario');

%% selecting scenario
prompt = "selezionare scarica: ";
idxData = input(prompt);

switch idxData
    case 1 
        varIpf = Scenario.Reference1.var;
        simIn =  Scenario.Reference1.simulink;
    case 2
        varIpf =  Scenario.Reference2.var;
        simIn =  Scenario.Reference2.simulink;
    case 3
        varIpf =  Scenario.Reference3.var;
        simIn =  Scenario.Reference3.simulink;
    case 4
        varIpf =  Scenario.Reference4.var;
        simIn =  Scenario.Reference4.simulink;
end


