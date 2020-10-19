%% initial
p1 = [1,0,1];
p2 = [sqrt(2)/2, sqrt(2)/2, 1.2];
L3 = 1;
L2 = L3;
L1 = L2;


%% calculations of a points on motion vector

% attempt to calculate coordinates using a line equation
% xy_a = p2(2) - (p2(2) - p1(2)) * p1(1)/ (p2(1) - p1(1));
% xy_b = (p2(2) - p1(2)) / (p2(1) - p1(1));
% xz_a = p2(3) - (p2(3) - p1(3)) * p1(1) / (p2(1) - p1(1));
% xz_b = (p2(3) - p1(3)) / (p2(1) - p1(1));

vector_line = p2 - p1;
delta = vector_line / (10 * length(vector_line));
scale = vector_line / delta;
counter = 0:1:scale;
needed_vector = p1 + counter' * delta;
all_thetas = ones(length(needed_vector), 3);

%% calculation of joints position for every point
for i = 1:length(needed_vector)
    thetas = RRRInverse(needed_vector(i,1), needed_vector(i,2), needed_vector(i,3), L1, L2, L3);
    all_thetas(i,1) = thetas(1,1);
    all_thetas(i,2) = thetas(1,2);
    all_thetas(i,3) = thetas(1,3);
end

%% A loop to reach every consecutive point of a line

%% initial
q10 = all_thetas(1, 1); 
q1f = all_thetas(2, 1); 
v1m = 1;
a1m = 10;
q20 = all_thetas(1, 2); 
q2f = all_thetas(2, 2); 
v2m = 1;
a2m = 10;
q30 = all_thetas(1, 3); 
q3f = all_thetas(2, 3); 
v3m = 1; 
a3m = 10;
v10 = 0
v20 = 0;
v30 = 0;
delta_t = 0.01;
n = 0;
while (floor(delta_t * 10^n) ~= delta_t * 10^n)
    n = n + 1;
end

E = 1 * 10^-n;

%% loop
for i = 1:length(all_thetas)
    disp(i); disp(" iteration");
t1a = v1m / a1m;
if rem(t1a, delta_t) ~= 0
    t1a_new = round(t1a, n)+E;
else
    t1a_new = round(t1a, n);
end
t1f = (q1f - q10) / v1m + t1a_new;
if rem(t1f, delta_t) ~= 0
    t1f_new = round(t1f, n)+E;
else
    t1f_new = round(t1f, n);
end

t2a = v2m / a2m;
if rem(t2a, delta_t) ~= 0
    t2a_new = round(t2a, n)+E;
else
    t2a_new = round(t2a, n);
end
t2f = (q2f - q20) / v2m + t2a_new;
if rem(t2f, delta_t) ~= 0
    t2f_new = round(t2f, n)+E;
else
    t2f_new = round(t2f, n);
end

t3a = v3m / a3m;
if rem(t3a, delta_t) ~= 0
    t3a_new = round(t3a, n)+E;
else
    t3a_new = round(t3a, n);
end
t3f = (q3f - q30) / v3m + t3a_new;
if rem(t3f, delta_t) ~= 0
    t3f_new = round(t3f, n)+E;
else
    t3f_new = round(t3f, n);
end

%% simultationus motion calculation
if (t3f_new >= t1f_new) && (t3f_new >= t2f_new)
    tf_new = t3f_new;
    ta_new = t3a_new;
elseif (t2f_new >= t1f_new) && (t2f_new >= t3f_new)
    tf_new = t2f_new;
    ta_new = t2a_new;
elseif (t1f_new >= t2f_new) && (t1f_new >= t3f_new)
    tf_new = t1f_new;
    ta_new = t1a_new;
end


v1_new = ((q1f - q10) / (tf_new - ta_new));
a1_new = v1_new / ta_new;

v2_new = ((q2f - q20) / (tf_new - ta_new));
a2_new = v2_new / ta_new;

v3_new = ((q3f - q30) / (tf_new - ta_new));
a3_new = v3_new / ta_new;

% joint 1 - coefficients:
% t0 --> ta:
a10 = q10;
a11 = v10;
a12 = 0.5 * a1_new;


% ta --> tf-ta:
a20 = q10 + 0.5 * a1_new * ta_new^2 - v1_new * ta_new;
a21 = v1_new;

% tf-ta --> tf:
a30 = q1f - 0.5 * a1_new * tf_new^2;
a31 = a1_new * tf_new;
a32 = -0.5 * a1_new;

% joint 2 - coefficients:
% t0 --> ta:
b10 = q20;
b11 = v20;
b12 = 0.5 * a2_new;


% ta --> tf-ta:
b20 = q20 + 0.5 * a2_new * ta_new^2 - v2_new * ta_new;
b21 = v2_new;

% tf-ta --> tf:
b30 = q2f - 0.5 * a2_new * tf_new^2;
b31 = a2_new * tf_new;
b32 = -0.5 * a2_new;

% joint 3 - coefficients:
% t0 --> ta:
c10 = q30;
c11 = v30;
c12 = 0.5 * a3_new;


% ta --> tf-ta:
c20 = q30 + 0.5 * a3_new * ta_new^2 - v3_new * ta_new;
c21 = v3_new;

% tf-ta --> tf:
c30 = q3f - 0.5 * a3_new * tf_new^2;
c31 = a3_new * tf_new;
c32 = -0.5 * a3_new;

        %% polynomial motion of 3 joints
    t = 0 : delta_t : tf_new;
    q1 = (a10+a11.*t+a12.*t.^2).*(t<=ta_new)...
        +(a20+a21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(a30+a31.*t+a32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
    v1 = (a11+2*a12.*t).*(t<=ta_new)...
        +(a21).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(a31+2*a32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
    acc1 = (2*a12).*(t<=ta_new)...
        +(0).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(2*a32).*(t>(tf_new-ta_new)).*(t<=tf_new);
    q2 = (b10+b11.*t+b12.*t.^2).*(t<=ta_new)...
        +(b20+b21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(b30+b31.*t+b32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
    v2 = (b11+2*b12.*t).*(t<=ta_new)...
        +(b21).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(b31+2*b32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
    acc2 = (2*b12).*(t<=ta_new)...
        +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
        +(2*b32).*(t>(tf_new-ta_new)).*(t<=tf_new);
    q3 = (c10+c11.*t+c12.*t.^2).*(t<=ta_new)...
        +(c20+c21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(c30+c31.*t+c32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
    v3 = (c11+2*c12.*t).*(t<=ta_new)...
        +(c21).*(t>ta_new).*(t<=(tf_new-ta_new))...
        +(c31+2*c32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
    acc3 = (2*c12).*(t<=ta_new)...
        +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
        +(2*c32).*(t>(tf_new-ta_new)).*(t<=tf_new);
% figure
% plot(t,q1,'k','linewidth',2)
% hold on
% plot(t,q2,'r','linewidth',2)
% hold on
% plot(t,q3,'b','linewidth',2)
% grid on
% title('position vs time')
%       rad2deg(all_thetas(i,1))

      %% next iteration initial
      if(i ~= length(all_thetas))
        q10 = all_thetas(i, 1);
        q1f = all_thetas(i + 1, 1);
        q20 = all_thetas(i, 2); 
        q2f = all_thetas(i + 1, 2);
        q30 = all_thetas(i, 3); 
        q3f = all_thetas(i + 1, 3);
        v10 = 0;
        v20 = 0;
        v30 = 0;
      end
end