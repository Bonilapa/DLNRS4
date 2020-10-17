function thetas = RRRInverse(x, y, z, L1, L2, L3)

theta1 = atan2(y, x);
if(z > L1)
    zz = z - L1;
else
    zz = L1 - z;
end
theta31 = double(acos((x^2 + zz^2 - L2^2 - L3^2) / (2 * L2 * L3)));
theta32 = double(-acos((x^2 + zz^2 - L2^2 - L3^2) / (2 * L2 * L3)));
theta21 = double(atan2(zz, x) - atan2((L3 * sin(theta31)) , (L2 + L3 * cos(theta31))));
theta22 = double(atan2(zz, x) - atan2((L3 * sin(theta32)) , (L2 + L3 * cos(theta32))));
thetas = [theta1, theta21, theta31;
            theta1, theta22, theta32];