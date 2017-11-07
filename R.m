function [ R_matrix ] = R( angle )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% This takes angles in the order roll, pitch, yaw (phi, theta, psi) and
% rotates them using Euler 3-1-2
phi = angle(1);
th = angle(2);
psi = angle(3);

% R(phi)
Rphi = [1,0,0; 0,cos(phi), sin(phi); 0, -sin(phi), cos(phi)];
% R(th)    
Rtheta = [cos(th),0,-sin(th); 0,1, 0; sin(th), 0, cos(th)];
% R(psi)    
Rpsi = [cos(psi),sin(psi),0; -sin(psi),cos(psi), 0; 0, 0, 1];

% %Euler 3-1-2 
% R_matrix= Rtheta*Rphi*Rpsi;
%Euler 3-2-1 
R_matrix= Rphi*Rtheta*Rpsi;

end