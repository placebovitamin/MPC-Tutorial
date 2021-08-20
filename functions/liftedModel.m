function [A_,B_] = liftedModel(A,B,N)
% [x_1;x_2;...;x_N] = A_ * x0 + B_ * [u_0;u_1;...;u_(N-1)]

A_ = cell2mat(cellfun(@(x)A^x,num2cell((1:N)'),'UniformOutput',0));
B_ =tril(cell2mat(cellfun(@(x)A^x,num2cell(toeplitz(0:N-1)),'UniformOutput',0)))*kron(eye(N),B);
           
end

