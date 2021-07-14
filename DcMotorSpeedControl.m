clc
clear all
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s = tf('s');

% open-loop transfer function
% using laplace transformation of dynamic equation
P_motor = K/((J*s+b)*(L*s+R)+K^2)

% motor_ss = ss(P_motor)%transfer function to state space conversion
%% Open loop response
figure(1)
step(P_motor, 0:0.1:5)
% linearSystemAnalyzer('step', P_motor, 0:0.1:5);
% rP_motor = 0.1/(0.5*s+1);

%% close loop response
Kp = 100;
C = pid(Kp);
sys_cl = feedback(C*P_motor,1)

t = 0:0.01:5;
figure(2)
step(sys_cl,t);
grid
title('Step Response with Proportional Control')
% controlSystemDesigner(P_motor)

%% PID Controller Design
Kp = 100;
Ki = 1;
Kd = 1;
C = pid(Kp,Ki,Kd);
sys_cl = feedback(C*P_motor,1)
figure(3)
step(sys_cl,t);
title('PID Control with Small Ki and Small Kd')

Kp = 100;
Ki = 200;
Kd = 1;
C = pid(Kp,Ki,Kd);
sys_cl = feedback(C*P_motor,1);
figure(4)
step(sys_cl,t)
grid
title('PID Control with Large Ki and Small Kd')

Kp = 100;
Ki = 200;
Kd = 10;
C = pid(Kp,Ki,Kd);
sys_cl = feedback(C*P_motor,1);
figure(5)
step(sys_cl,t)
grid
title('PID Control with Large Ki and Large Kd')

%% Root Locus Controller Design
controlSystemDesigner('rlocus', P_motor) 

%% Frequency Domain Methods for Controller Design
figure()
bode(P_motor)
grid
title('Bode Plot of the Original Plant')

figure()
[mag,phase,w] = bode(P_motor,10)

C = 72;
margin(C*P_motor);

sys_cl = feedback(C*P_motor,1);
t = 0:0.01:10;
step(sys_cl,t), grid
title('Step Response with Proportional Gain = 72')

C = 45*(s + 1)/(s + 0.01);
bode(C)
grid
title('Bode Plot of the Lag Compensator')

sys_cl = feedback(C*P_motor,1);
t = 0:0.01:10;
step(sys_cl,t), grid
title('Step Response with Lag Compensator')

%% State-Space Methods for Controller Design
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   0];
D = 0;
sys = ss(A,B,C,D);

sys_order = order(sys)
sys_rank = rank(ctrb(A,B))

p1 = -5 + 1i;
p2 = -5 - 1i;
Kc = place(A,B,[p1 p2])

t = 0:0.01:3;
sys_cl = ss(A-B*Kc,B,C,D);
step(sys_cl,t)
grid
title('Step Response with State-Feedback Controller')

Nbar = rscale(sys,Kc)

t = 0:0.01:10;
step(sys_cl*Nbar,t)
grid
title('Step Response with State-Feedback Controller and Precompensator')

%% Digital Controller Design
P_motor = K/((J*s+b)*(L*s+R)+K^2);
zpk(P_motor)

Ts = 0.05;
dP_motor = c2d(P_motor,Ts,'zoh');
zpk(dP_motor)

sys_cl = feedback(dP_motor,1);
[y,t] = step(sys_cl,12);
stairs(t,y);
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
title('Stairstep Response: Original')

Kp = 100;
Ki = 200;
Kd = 10;
C = Kp + Ki/s + Kd*s;
dC = c2d(C,Ts,'tustin')

sys_cl = feedback(dC*dP_motor,1);
[x2,t] = step(sys_cl,12);
stairs(t,x2)
xlabel('Time (seconds)')
ylabel('Velocity (rad/s)')
title('Stairstep Response: with PID controller')

rlocus(dC*dP_motor)
axis([-1.5 1.5 -1 1])
title('Root Locus of Compensated System')

z = tf('z',Ts);
dC = dC/(z+0.82);
rlocus(dC*dP_motor);
axis([-1.5 1.5 -1 1])
title('Root Locus of Compensated System');

[K,poles] = rlocfind(dC*dP_motor)

sys_cl = feedback(0.8*dC*dP_motor,1);
[x3,t] = step(sys_cl,8);
stairs(t,x3)
xlabel('Time (seconds)')
ylabel('Velocity (rad/s)')
title('Stairstep Response: with Modified PID controller')
