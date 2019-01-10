%% State Space representation
A=[-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B=[0.232; 0.0203; 0];
C=[0 0 1];
D=[0];
state_space=ss(A,B,C,D) %State Space representation of system
%% Open Loop Transfer Function and Analysis
[num,den]=ss2tf(A,B,C,D,1); %To convert state space model to transfer function
open_loop_tf=tf(num,den)
t=[0:0.01:15];
figure(1);
subplot(2,1,1);
impulse(open_loop_tf,t); %Computing open loop impulse response
xlabel('Time');
ylabel('Pitch angle(rads)');
title('Open Loop impulse response');
subplot(2,1,2);
step(open_loop_tf,t); %Computing open loop step response
xlabel('Time');
ylabel('Pitch angle(rads)');
title('Open Loop step response');
figure(2);
pzplot(open_loop_tf);
%% Closed Loop Response
pole(open_loop_tf)
sys_cl = feedback(open_loop_tf,1)
figure(3);
subplot(2,1,1);
impulse(sys_cl); %Computing closed loop Impulse response
xlabel('Time');
ylabel('Pitch angle(rads)');
title('Closed Loop Impulse response');
subplot(2,1,2);
step(sys_cl); %Computing closed loop Step response
xlabel('time');
ylabel('pitch angle (rad)');
title('Closed-loop Step Response');
poles = pole(sys_cl);
zeros = zero(sys_cl);
s = tf('s');
R = 1/s;
Y = zpk(sys_cl*R);
%[r,p,k] = residue([1.151 0.1774],[1 0.739 2.072 0.1774 0])
%[num,den] = residue(r(1:2),p(1:2),k);
%tf(num,den)
%% PID Controller Design
controlSystemDesigner(open_loop_tf)
%% Controllability & Observability
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];
co = ctrb(A,B);
Controllability = rank(co)
%% LQR method
p = 590;
Q = p*C'*C
R = 1;
[K] = lqr(A,B,Q,R)
Nbar = rscale(A,B,C,D,K)
sys_cl_1 = ss(A-B*K, B*Nbar, C, D)
figure(4);
subplot(2,1,1);
step(sys_cl_1);
ylabel('pitch angle (rad)');
xlabel('Time(sec)');
title('Closed-Loop Step Response: LQR with Precompensation');
%% Digital Control System
sys_ss = ss(A,B,C,D);
Ts = 1/100;
sys_d = c2d(sys_ss,Ts,'zoh')
%% Controllability of Digital Sysstem
co = ctrb(sys_d);
Controllability = rank(co) % Controllability of digital system
%% LQR Method for Digital Control System
A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;
p = 50;
Nbar_d = 6.95;
Q = p*C'*C
R = 1;
[K] = dlqr(A,B,Q,R)
time = 0:0.01:10;
theta_des = 0.2*ones(size(time));
sys_cl_d = ss(A-B*K,B*Nbar_d,C,D,Ts);
[y,t] = lsim(sys_cl_d,theta_des);
figure(4);
subplot(2,1,2);
stairs(t,y)
xlabel('time (sec)');
ylabel('pitch angle (rad)');
title('Closed-Loop Step Response: DLQR with Precompensation');
%% R-scale function
function[Nbar]=rscale(a,b,c,d,k)
% Given the single-input linear system:
%       .
%       x = Ax + Bu
%       y = Cx + Du
% and the feedback matrix K,
% the function rscale(sys,K) or rscale(A,B,C,D,K)
% finds the scale factor N which will
% eliminate the steady-state error to a step reference
% for a continuous-time, single-input system
% with full-state feedback using the schematic below:
%
%                         /---------\
%      R         +     u  | .       |
%      ---> N --->() ---->| X=Ax+Bu |--> y=Cx ---> y
%                -|       \---------/
%                 |             |
%                 |<---- K <----|
%
error(nargchk(2,5,nargin));
% --- Determine which syntax is being used ---
nargin1 = nargin;
if (nargin1==2),	% System form
    [A,B,C,D] = ssdata(a);
    K=b;
elseif (nargin1==5), % A,B,C,D matrices
    A=a; B=b; C=c; D=d; K=k;
else error('Input must be of the form (sys,K) or (A,B,C,D,K)')
end;
% compute Nbar
s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;C,D])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + K*Nx;
end