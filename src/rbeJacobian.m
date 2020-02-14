syms q1 q2 q3 l1 l2 l3;
% l1 = 135;
% l2 = 175;
% l3 = 169.28;

T01 = [cos(q1), 0, sin(q1), 0;
    sin(q1), 0, -cos(q1), 0;
    0, 1, 0, l1;
    0, 0, 0, 1;];
T12 = [cos(q2), -sin(q2), 0, l2*cos(q2);
    sin(q2) cos(q2), 0, l2*sin(q2);
    0, 0, 1, 0;
    0, 0, 0, 1;];
T23 = [sin(q3), cos(q3), 0, l3*sin(q3);
    -cos(q3), sin(q3), 0, -1*l3*cos(q3);
    0, 0, 1, 0;
    0, 0, 0, 1;];
T02= T01*T12;
disp('T02')
disp(T02)
T03=T02*T23;
disp('T03')
disp(T03)
Jp1= cross(T01(1:3,3),T03(1:3,4)- T01(1:3,4));
Jp2= cross(T02(1:3,3)',(T03(1:3,4)- T02(1:3,4))');
Jp3= cross(T03(1:3,3)',(T03(1:3,4)- T03(1:3,4))');
Jp=[Jp1;Jp2;Jp3;];
disp('Jp')
disp(Jp1)