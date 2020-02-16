function [ dq ] = numericIKAlgo( Jp, pi, pf )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

iJp = inv(Jp);
pE = (pf - pi)';

disp('Pe');
disp(pE)

dq = (iJp * pE)';


end

