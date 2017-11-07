function [ R_matrix ] = Rrel( angle )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% This takes angles and rotate it by yaw and then pitch into new frame
th = angle(1);
psi = angle(2);

% R(th)    
Rtheta = [cos(th),0,-sin(th); 0,1, 0; sin(th), 0, cos(th)];
% R(psi)    
Rpsi = [cos(psi),sin(psi),0; -sin(psi),cos(psi), 0; 0, 0, 1];

%Euler 3-1-2 
R_matrix= Rpsi*Rtheta;
end