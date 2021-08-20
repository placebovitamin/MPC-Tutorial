%% Clear and Addpath
close all;
clear all;
addpath functions;
addpath matlab2tikz;

%% General
T = 0.05;
T_sim = 5;
steps = floor(T_sim/T);

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
U_set.A = [-1;-1];
U_set.b = [20;20];
% X_set.A = [-1,0;1,0;0,-1;0,1];
% X_set.b = [10;10;10;10];
X_set = Polyhedron('A',[-1,0;1,0;0,-1;0,1],'b',[10;10;10;10]);
% terminal constraint
X_f = ControlInvariantSet( A, B, Polyhedron('A',X_set.A,'b',X_set.b), Polyhedron('A',U_set.A,'b',U_set.b), 'maxItr',100);
[X_f] = sortPolyhedron(X_f); % sort vertices of polyhedron because of tikz problem

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

figure(1)
subplot(2,2,1)
hold on
X_f.plot('alpha',0.05)
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (1, X_log,X_pre)
subplot(2,2,2)
hold on
X_f.plot('alpha',0.05)
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (5, X_log,X_pre)
subplot(2,2,3)
hold on
X_f.plot('alpha',0.05)
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (25, X_log,X_pre)
subplot(2,2,4)
hold on
X_f.plot('alpha',0.05)
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (i, X_log,X_pre)

matlab2tikz('Example3.tex')

%% plot input
fig = figure(2); 
fig.Position = fig.Position .* [1,1,1,0.5]; 

stairs(0:length(U_log)-1,U_log,'LineWidth',1)

xlabel('Iterations')
ylabel('$u$','interpreter','latex')
axis([-1,100,-21,5])

matlab2tikz('Example3_input.tex')


