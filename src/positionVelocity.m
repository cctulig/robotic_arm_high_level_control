function [ dP ] = positionVelocity( q dQ )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
J=jacob0(q);
dP=J*dQ;

end

