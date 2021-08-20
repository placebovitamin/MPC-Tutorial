%% Clear and Addpath
close all;
clear all;
addpath functions;
addpath matlab2tikz;

%% General
T = 0.05;
T_sim = 5;
steps =5;% floor(T_sim/T);

%% prediction horizon
N  = 5;

%% Model - Disretized double integrater
A = [1,T;0,1];             % System Matirx
B = T* [0;1];          % Input Matrix
n = size(A,2);          % dimention of state space
m = size(B,2);          % dimention of input space
x0 = [7.24;10];             % initial state
%x0 = [0;10];             % initial state

%% Constraints A*x < b
U_set.A = [1;-1];
U_set.b = [20;20];
% X_set.A = [-1,0;1,0;0,-1;0,1];
% X_set.b = [10;10;10;10];
X_set = Polyhedron('A',[-1,0;1,0;0,-1;0,1],'b',[10;10;10;10]);
% terminal constraint
X_f   = X_set;

%% Optimization weights
Q  = eye(n);
R  = eye(m);
[Qf,~,~] = idare(A,B,Q,R,[],[]);

%% Preprocessing
Uad = admissibleInputs(A,B,X_set,U_set,X_f,N);

%% Variable declaration for logging
X_log = zeros(n,steps);
U_log = zeros(m,steps);
X_pre = zeros(n,N,steps);

%% Simulation
x=x0;
for i = 1:steps
    % Control Method: MPC
    U = MPC_opt(x,A,B,Q,Qf,R,Uad,N);
    u = U(1:m);
    
    % Prediction
    [A_,B_] = liftedModel(A,B,N);
    X =reshape( A_*x + B_*U, 2 ,N);
    
    % Log
    X_log(:,i) = x;
    U_log(:,i) = u;
    X_pre(:,:,i) = X;
    
    % System
    x=A*x+B*u;
end


%% plot
fig = figure(1);
fig.Position = [763 508 560 180];
subplot(1,2,1)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (4, X_log,X_pre)
subplot(1,2,2)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (i, X_log,X_pre)


matlab2tikz('Example2.tex')

