prompt = "Select controller (1 = PID) (2 = GA PID) : ";
PIDtuning = input(prompt);

%% automatic tuning

if PIDtuning == 1
    opts = pidtuneOptions('PhaseMargin',65);
    wc = 200;
    
    R1 = pidtune(sys(1,1)*PSsys,'PIDF',wc,opts);
    R2 = pidtune(sys(2,2)*PSsys,'PIDF',wc,opts);
    R3 = pidtune(sys(3,3)*PSsys,'PIDF',wc,opts);
    R4 = pidtune(sys(4,4)*PSsys,'PIDF',wc,opts);
    R5 = pidtune(sys(5,5)*PSsys,'PIDF',wc,opts);
    R6 = pidtune(sys(6,6)*PSsys,'PIDF',wc,opts);
    R7 = pidtune(sys(7,7)*PSsys,'PIDF',wc,opts);
    R8 = pidtune(sys(8,8)*PSsys,'PIDF',wc,opts);
    R9 = pidtune(sys(9,9)*PSsys,'PIDF',wc,opts);
    R10 = pidtune(sys(10,10)*PSsys,'PIDF',wc,opts);
    R11 = pidtune(sys(11,11)*PSsys,'PIDF',wc,opts);
    R12 = pidtune(sys(12,12)*PSsys,'PIDF',wc,opts);
else
    load("OptimalTunedGA.mat");

    R1 = pid(Controller.R1);
    R2 = pid(Controller.R2);
    R3 = pid(Controller.R3);
    R4 = pid(Controller.R4);
    R5 = pid(Controller.R5);
    R6 = pid(Controller.R6);
    R7 = pid(Controller.R7);
    R8 = pid(Controller.R8);
    R9 = pid(Controller.R9);
    R10 = pid(Controller.R10);
    R11 = pid(Controller.R11);
    R12 = pid(Controller.R12);
end

Controller = struct('R1',R1,'R2',R2,'R3',R3,'R4',R4,'R5',R5,'R6',R6, ...
    'R7',R7,'R8',R8,'R9',R9,'R10',R10,'R11',R11,'R12',R12);

prompt = "controller update correctly";

open_system("ControlSystem.slx");