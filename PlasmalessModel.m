%% load the data

%% selecting scenario
load('Scenario.mat')
prompt = "select scenario: ";
idxData = input(prompt);
dt=1e-3;

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

% load system data
load('Equil_shot_EDDY_Plasmaless_2022_CL.mat')             

%% Plasmaless model

% inductances, resistences and voltages matrices
circuitNumber = LinearModel.PoloidalCircuits.Number-1;  % number of Poloidal Circuit used in the model

L = LinearModel.L;                                      % Inductance matrix 
R = LinearModel.R;                                      % Resistence matrix
L = L(1:end-1,1:end-1);                                 % Inductance model matrix
R = R(1:end-1,1:end-1);                                 % Resistence model matrix
S = LinearModel.S(1:end-1,1:circuitNumber);             % Voltage model matrix

% system matices for state-space mimo model
A = -inv(L)*R;                                          % state matrix                                          
B = L\S;                                                % input matrix
C = eye(circuitNumber,length(A));                       % output matrix
D = zeros(circuitNumber,circuitNumber);                 % feedforward matrix

% mimo system in state-space form
sys = ss(A,B,C,D);
sys.StateName = LinearModel.OutputsInfo.Name(1:76);
sys.Name = 'Plasmaless Circuit';
sys.InputName = LinearModel.InputsInfo.Name(1:12);
sys.OutputName = LinearModel.OutputsInfo.Name(1:12);

% power supply
PSnum = 1;                                              % transfer function numerator  
PSden = [7e-3 1];                                       % transfer function denominator
PSdelay = 1.67e-3;                                      % delay of each power supply
PSsat = [350 350 350 350 350 350 ...                  
         800 800 400 400 330 330];                      % abs of saturation limit 

PSsys = tf(PSnum,PSden);                                % Powe Supply Transfer function
PSsys.InputDelay = PSdelay;                             % PS delay
PSsys.Name = 'Power Supply';
PSsys.InputName = 'Vpf';
PSsys.InputUnit = 'volt';
PSsys.OutputName = 'V';
PSsys.OutputUnit = 'volt';

padeOrder = 2;
PSpade = pade(PSsys,padeOrder);                         % pade approx of Power Supply

% relative gain array to see if the system can be decoupled
% for a centralized controller
RGA = dcgain(sys).*inv(dcgain(sys))';

% reduced systems from input i and output i
ord = 2;                                                % reduction order
sys1 = balred(sys(1,1),ord);
sys2 = balred(sys(2,2),ord);
sys3 = balred(sys(3,3),ord);
sys4 = balred(sys(4,4),ord);
sys5 = balred(sys(5,5),ord);
sys6 = balred(sys(6,6),ord);
sys7 = balred(sys(7,7),ord);
sys8 = balred(sys(8,8),ord);
sys9 = balred(sys(9,9),ord);
sys10 = balred(sys(10,10),ord);
sys11 = balred(sys(11,11),ord);
sys12 = balred(sys(12,12),ord);

PlasmalessModelMIMO = struct('A',A,'B',B,'C',C,'D',D,'sys',sys);
PlasmalessModelSISO = struct('sys1',sys1,'sys2',sys2,'sys3',sys3,'sys4',sys4,'sys5',sys5, ...
    'sys6',sys6,'sys7',sys7,'sys8',sys8,'sys9',sys9,'sys10',sys10,'sys11',sys11,'sys12',sys12);
PowerSupply = struct('PSsys',PSsys,'PSsat',PSsat,'PSpade',PSpade);

%% selecting controller
prompt = "select controller type (you have to change also variant subsystem) (1 = PID) (2 = Rlocus) : ";
controllerType = input(prompt);

switch controllerType
    case 1
        run("PIDController.m")
    case 2
        run("RLocusController.m")
end
