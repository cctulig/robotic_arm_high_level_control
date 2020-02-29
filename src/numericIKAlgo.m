function [ dq ] = numericIKAlgo( qi, Pf )
%numericIKAlgo Inverse Differential Kinematics Calculations

% Gain factor
G = eye(3,3).*[.25;.25;.25];

% Jacobian/Inverse Position Jacobian
J = jacob0(qi);
Jp = J(1:3,1:3);
Jpi = pinv(Jp);

% Change in position
Pi = fwkin3001(qi(1), qi(2), qi(3));
Pdelta = (Pf' - Pi);

% Calculate change in angle
dq = (Jpi * G * Pdelta)';

end

