function U = MPC_opt(x0,A,B,Q,Qf,R,Uad,N)
% Optimization 
%  J = 1/2 * U'*(B_'*Q_*B_ +R_)*U  + (B_'*Q_*A_*x0 )' * U ; 

% Lifted dynamics: [x_1;x_2;...;x_N] = A_ * x0 + B_ * [u_0;u_1;...;u_(N-1)]
[A_,B_] = liftedModel(A,B,N);

% Optimation Weights
Q_ = blkdiag(kron(eye(N-1),Q),Qf);
H = B_'* Q_* B_ +   kron(eye(N),R);
f = B_'*Q_*A_*x0 ;

% Optimization
options = mpcActiveSetOptions;
iA0 = false(size(Uad.b));
[U,exitflag] = mpcActiveSetSolver(H,f,Uad.A,Uad.b-Uad.B*x0,Uad.Ae,Uad.be,iA0,options);

% Error handling
if exitflag<=0
    disp(exitflag);
    error('problem solving qp');
    % warning('problem solving qp');
    % U = zeros(obj.nu*obj.Nmpc,1);
end

end
