function H = RRRFNum(theta11, theta22, theta33, L1, L2, L3)
syms theta1 theta2 theta3 real
H = RRRF(L1, L2, L3);
H = subs(H, {theta1, theta2, theta3}, {theta11, theta22, theta33});
H = double(H);