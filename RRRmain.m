L3 = 1;
L2 = L3;
L1 = L2;

thetas = [0, 0, deg2rad(45)];
% H = RRRFNum(thetas(1), thetas(2), thetas(3), L1, L2, L3);
% thetas2 = rad2deg(RRRInverse(H(1,4), H(2,4), H(3,4), L1, L2, L3));

%% Task 1
J = RRRJacobian(L1, L2, L3);
syms theta1 theta2 theta3 real
% JNum = double(subs(J, {theta1, theta2, theta3}, {thetas(1), thetas(2), thetas(3)}));

%% Task 2
t0 = 0; 
tf = 2;

% 1st joint
q10 = 8; 
q1f = 2;
v10 = 0; 
v1f = 0;
acc10 = 0; 
acc1f = 0;

A = [1 t0 t0^2 t0^3 t0^4 t0^5
      0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
      0 0 2 6*t0 12*t0^2 20*t0^3
      1 tf tf^2 tf^3 tf^4 tf^5
      0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
      0 0 2 6*tf 12*tf^2 20*tf^3];
c = [q10; v10; acc10; q1f; v1f; acc1f];
b = A\c;
% assign the results to the coefficients 
a0 = b(1); a1 = b(2); a2 = b(3); a3 = b(4); a4 = b(5); a5 = b(6);

t = 0:0.1:5;
qt = a0 + a1.* t + a2.* t.^2 + a3.* t.^3 + a4.* t.^4 + a5.* t.^5;
vt = 5*a5.*t.^4 + 4*a4.*t.^3 + 3*a3.*t.^2 + 2*a2.*t + a1;
acct = 20*a5.*t.^3 + 12*a4.*t.^2 + 6*a3.*t + 2*a2;

figure
 plot(t,qt,'g-')
 title('position vs time')
 grid on
 figure
 plot(t,vt,'b-')
 title('velocity vs time')
 grid on
 figure
 plot(t,acct,'k-')
 title('acceleration vs time')
 grid on
 
% 2st joint
q10 = 8; 
q1f = 3;
v10 = 0; 
v1f = 0;
acc10 = 0; 
acc1f = 0;

A = [1 t0 t0^2 t0^3 t0^4 t0^5
      0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
      0 0 2 6*t0 12*t0^2 20*t0^3
      1 tf tf^2 tf^3 tf^4 tf^5
      0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
      0 0 2 6*tf 12*tf^2 20*tf^3];
c = [q10; v10; acc10; q1f; v1f; acc1f];
b = A\c;
% assign the results to the coefficients 
a0 = b(1); a1 = b(2); a2 = b(3); a3 = b(4); a4 = b(5); a5 = b(6);

t = 0:0.1:5;
qt = a0 + a1.* t + a2.* t.^2 + a3.* t.^3 + a4.* t.^4 + a5.* t.^5;
vt = 5*a5.*t.^4 + 4*a4.*t.^3 + 3*a3.*t.^2 + 2*a2.*t + a1;
acct = 20*a5.*t.^3 + 12*a4.*t.^2 + 6*a3.*t + 2*a2;

figure
 plot(t,qt,'g-')
 title('position vs time')
 grid on
 figure
 plot(t,vt,'b-')
 title('velocity vs time')
 grid on
 figure
 plot(t,acct,'k-')
 title('acceleration vs time')
 grid on
 
% 3rd joint
q10 = 8; 
q1f = 4;
v10 = 0; 
v1f = 0;
acc10 = 0; 
acc1f = 0;

A = [1 t0 t0^2 t0^3 t0^4 t0^5
      0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
      0 0 2 6*t0 12*t0^2 20*t0^3
      1 tf tf^2 tf^3 tf^4 tf^5
      0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
      0 0 2 6*tf 12*tf^2 20*tf^3];
c = [q10; v10; acc10; q1f; v1f; acc1f];
b = A\c;
% assign the results to the coefficients 
a0 = b(1); a1 = b(2); a2 = b(3); a3 = b(4); a4 = b(5); a5 = b(6);

t = 0:0.1:5;
qt = a0 + a1.* t + a2.* t.^2 + a3.* t.^3 + a4.* t.^4 + a5.* t.^5;
vt = 5*a5.*t.^4 + 4*a4.*t.^3 + 3*a3.*t.^2 + 2*a2.*t + a1;
acct = 20*a5.*t.^3 + 12*a4.*t.^2 + 6*a3.*t + 2*a2;

figure
 plot(t,qt,'g-')
 title('position vs time')
 grid on
 figure
 plot(t,vt,'b-')
 title('velocity vs time')
 grid on
 figure
 plot(t,acct,'k-')
 title('acceleration vs time')
 grid on