s = tf('s');

%% power supply and plasmaless reducted SISO models series
G1 = sys1 * PSpade; 
G2 = sys2 * PSpade;
G3 = sys3 * PSpade;
G4 = sys4 * PSpade;
G5 = sys5 * PSpade;
G6 = sys6 * PSpade;
G7 = sys7 * PSpade;
G8 = sys8 * PSpade;
G9 = sys9 * PSpade;
G10 = sys10 * PSpade;
G11 = sys11 * PSpade;
G12 = sys12 * PSpade;

%% tuning Rlocus controller
target = 205 * pade(exp(-PSdelay*s),padeOrder) / s ;    % target open loop function
ControllerOrder = 3;

if ControllerOrder == 3
    R1 = tf(minreal(target / G1));
    R2 = tf(minreal(target / G2));
    R3 = tf(minreal(target / G3));
    R4 = tf(minreal(target / G4));
    R5 = tf(minreal(target / G5));
    R6 = tf(minreal(target / G6));
    R7 = tf(minreal(target / G7));
    R8 = tf(minreal(target / G8));
    R9 = tf(minreal(target / G9));
    R10 = tf(minreal(target / G10));
    R11 = tf(minreal(target / G11));
    R12 = tf(minreal(target / G12));
else 
    R1 = balred(tf(minreal(target / G1)),2);
    R2 = balred(tf(minreal(target / G2)),2);
    R3 = balred(tf(minreal(target / G3)),2);
    R4 = balred(tf(minreal(target / G4)),2);
    R5 = balred(tf(minreal(target / G5)),2);
    R6 = balred(tf(minreal(target / G6)),2);
    R7 = balred(tf(minreal(target / G7)),2);
    R8 = balred(tf(minreal(target / G8)),2);
    R9 = balred(tf(minreal(target / G9)),2);
    R10 = balred(tf(minreal(target / G10)),2);
    R11 = balred(tf(minreal(target / G11)),2);
    R12 = balred(tf(minreal(target / G12)),2);
end

%% fractal decomposition for anti windup
[r1,p1,k1] = residue(R1.Numerator{1,1},R1.Denominator{1,1});
fil1 = zpk(minreal(R1 - k1 - r1(end)/s, 1e-2));

[r2,p2,k2] = residue(R2.Numerator{1,1},R2.Denominator{1,1});
fil2 = zpk(minreal(R2 - k2 - r2(end)/s, 1e-2));

[r3,p3,k3] = residue(R3.Numerator{1,1},R3.Denominator{1,1});
fil3 = zpk(minreal(R3 - k3 - r3(end)/s, 1e-2));

[r4,p4,k4] = residue(R4.Numerator{1,1},R4.Denominator{1,1});
fil4 = zpk(minreal(R4 - k4 - r4(end)/s, 1e-2));

[r5,p5,k5] = residue(R5.Numerator{1,1},R5.Denominator{1,1});
fil5 = zpk(minreal(R5 - k5 - r5(end)/s, 1e-2));

[r6,p6,k6] = residue(R6.Numerator{1,1},R6.Denominator{1,1});
fil6 = zpk(minreal(R6 - k6 - r6(end)/s, 1e-2));

[r7,p7,k7] = residue(R7.Numerator{1,1},R7.Denominator{1,1});
fil7 = zpk(minreal(R7 - k7 - r7(end)/s, 1e-2));

[r8,p8,k8] = residue(R8.Numerator{1,1},R8.Denominator{1,1});
fil8 = zpk(minreal(R8 - k8 - r8(end)/s, 1e-2));

[r9,p9,k9] = residue(R9.Numerator{1,1},R9.Denominator{1,1});
fil9 = zpk(minreal(R9 - k9 - r9(end)/s, 1e-2));

[r10,p10,k10] = residue(R10.Numerator{1,1},R10.Denominator{1,1});
fil10 = zpk(minreal(R10 - k10 - r10(end)/s, 1e-2));

[r11,p11,k11] = residue(R11.Numerator{1,1},R11.Denominator{1,1});
fil11 = zpk(minreal(R11 - k11 - r11(end)/s, 1e-2));

[r12,p12,k12] = residue(R12.Numerator{1,1},R12.Denominator{1,1});
fil12 = zpk(minreal(R12 - k12 - r12(end)/s, 1e-2));

Controller = struct('R1',R1,'R2',R2,'R3',R3,'R4',R4,'R5',R5,'R6',R6, ...
    'R7',R7,'R8',R8,'R9',R9,'R10',R10,'R11',R11,'R12',R12);

prompt = "controller update correctly";

open_system("ControlSystem.slx");