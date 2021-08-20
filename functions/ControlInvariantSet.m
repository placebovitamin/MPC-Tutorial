function [Xf] = ControlInvariantSet( A, B, X, U , varargin)
% Computes the maximum Control Invariant Set starting from the origin
%
% Inputs
% A: System matrix
% B: Input matrix
% X: Polyhedron for x
% U: Polyhedron for u
% option: Procciding if algroithm dosen not converge:
%         approx: Use a control invriant Set as approximation for the
%         maximum control invariant set
%         origin: Use origin as  terminal constraint
%         else:   Use outcome of the abort calculation
%
% Outpus
% Xf: control invariant set
% isMax: (Boolean) true if method is converaged

validationScalar = @(x) validateattributes(x,{'numeric'},{'scalar'});
validationLogic = @(x) validateattributes(x,{'logical'},{'scalar'});
p = inputParser;
addParameter(p,'maxItr',20,validationScalar);
addParameter(p,'print',true,validationLogic);
p.parse(varargin{:});

maxItr       = p.Results.maxItr;
print        = p.Results.print;

converged = false;

f = waitbar(0,'Please wait...');

Xf = Polyhedron('Ae',eye(size(A)),'be',zeros(size(A,1),1)); % only origin
for i = 1:maxItr
    if isvalid(f)
        waitbar(i/maxItr,f,[num2str(i),' / ',num2str(maxItr)]);
    end
    Xf2 = PreSet(A,B, Xf, U);
    Xf2=Xf2.intersect(X);
    if Xf2==Xf
        converged = true;
        break
    else
        Xf = Xf2;
    end
end
Xf = Xf.intersect(X).minHRep();
if print
    if converged
        fprintf('Iterations for control invariant set is converged: %d\n', i);
    else
        fprintf('Iterations for control invariant set is not converged: %d\n', i);
    end
end
if isvalid(f)
    delete(f);
end

end

function Pre = PreSet(A,B,X,U)
Pre = invAffineMap(X+(-B*U),A);
Pre.minHRep;
end

