function [ dq ] = numericIKAlgo( qi, Pf )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

G = eye(3,3).*[.1;.1;.1];
J = jacob0(qi);
Jp = J(1:3,1:3);
Jpi = pinv(Jp);

Pi = fwkin3001(qi(1), qi(2), qi(3));
Pdelta = (Pf' - Pi);

dq = (Jpi * G * Pdelta)';
%ikin w/ jacobian -> get positon, get jacobian (3,3) invert, multiply by
%deriv of final pos and ^ pos
%


end

