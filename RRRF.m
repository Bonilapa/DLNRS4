function H = RRRF(L1, L2, L3)

syms theta1 theta2 theta3 real

H = Rz(theta1) * Tz(L1) * Ry(theta2) * Tx(L2) * Ry(theta3) * Tx(L3);
H = simplify(H);