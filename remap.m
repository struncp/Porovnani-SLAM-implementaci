function y = remap(b,ac,xz)
%premapuje hodnoty b z [a c] do [x z]

a = ac(1);
c = ac(2);
x = xz(1);
z = xz(2);

y = (b - a) * (z - x) / (c - a) + x;