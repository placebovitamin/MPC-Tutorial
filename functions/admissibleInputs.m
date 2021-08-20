function Uad = admissibleInputs(A,B,X_set,U_set,X_f,N)
% A_*x0 + B_* U \in Prod(X_set) 
% Prod(X_set).A (A_*x0 + B_* U ) \leq Prod(X_set).b
% Prod(X_set).A*B_  * U \leq Prod(X_set).b-Prod(X_set).A*A_*x0 
% Return values: 
% Uab = { U | Uab.A * U < Uab.b - Uab.B*x0 }

n=size(A,1);
[A_,B_] = liftedModel(A,B,N);


Uad.A = [kron(eye(N),U_set.A);blkdiag(kron(eye(N-1),X_set.A),X_f.A)*B_];
Uad.b = [kron(ones(N,1),U_set.b);[kron(ones(N-1,1),X_set.b);X_f.b]];
Uad.B = [zeros(N*size(U_set.b,1),n) ; blkdiag(kron(eye(N-1),X_set.A),X_f.A)*A_];
Uad.Ae = zeros(0,size(B_,2));
Uad.be = zeros(0,1);

end

