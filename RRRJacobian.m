function J = RRRJacobian(L1, L2, L3)
syms theta1 theta2 theta3 real
H = RRRF(L1, L2, L3);
R = simplify(H(1:3, 1:3));
invR = inv(R);
O = [invR, zeros(3,1); 0 0 0 1];

%% theta1
Jt1r = Rzd(theta1) * Tz(L1) * Ry(theta2) * Tx(L2) * Ry(theta3) * Tx(L3) * O;
Jt1r = simplify(Jt1r);
J1 = [Jt1r(1,4), Jt1r(2,4), Jt1r(3,4), Jt1r(3,2), Jt1r(1,3), Jt1r(2,1)];

%% theta2
Jt2r = Rz(theta1) * Tz(L1) * Ryd(theta2) * Tx(L2) * Ry(theta3) * Tx(L3) * O;
Jt2r = simplify(Jt2r);
J2 = [Jt2r(1,4), Jt2r(2,4), Jt2r(3,4), Jt2r(3,2), Jt2r(1,3), Jt2r(2,1)];

%% theta3
Jd3r = Rz(theta1) * Tz(L1) * Ry(theta2) * Tx(L2) * Ryd(theta3) * Tx(L3) * O;
Jd3r = simplify(Jd3r);
J3 = [Jd3r(1,4), Jd3r(2,4), Jd3r(3,4), Jd3r(3,2), Jd3r(1,3), Jd3r(2,1)];

J = [simplify(J1); simplify(J2); simplify(J3)]';
