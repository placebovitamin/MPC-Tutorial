function [Pout] = sortPolyhedron(Pin)
V = Pin.V;
X = V(:,1);
Y = V(:,2);
phi =atan2(Y,X);

[~,sortIdx] = sort(phi);

Pout = Polyhedron('V',[X(sortIdx),Y(sortIdx)]);

end

