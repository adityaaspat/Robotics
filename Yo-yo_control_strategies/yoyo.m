clc 
clear all
% Parameters
m = 0.1;    % Mass of the yoyo
g = 9.81;   % Acceleration due to gravity
r = 0.035;   % Radius of the yoyo
e = 0.002;   % Frictional coefficient in the string
I = 0.0306; % Moment of inertia of the yoyo
x0=[0; 0];
% System matrices
A = [0 ,1; 0, -(e*r)/(I + m*r^2)];
B = [0; 1/(I + m*r^2)];
C = [1 ,0];
D = 0;
des_poles=[0 0];
Bi=1/32.5494;
% Conversions
T=ctrb(A,B);
A_ccf=inv(T)*A*T;
B_ccf=inv(T)*B;
C_ccf=C*T;
%Computing L and K
K=acker(A,B,[0 0])
L=acker(A_ccf',C_ccf', [-0.01 -0.01])'
% tf=32.55/(s^2 + 0.002278*s);
x11=-255658390505305808600/47223803052363402509;
x21=361183241434822606848/7254263272660205;
x12=-255658390505305808600/47223803052363402509;
x22=2361183241434822606848/7254263272660205;
X=[x11 x12;x21 x22]