%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 673 Perception For Autonomous Robotics
% PROJECT 2  - Spring 2018
%
% Visual Odometry 
% FUNCTION - Fundamental_Matrix
% 
% It Normalizes the given array based on the mean distance and scale.
%
% 
% Code By: Mayank Pathak
%          115555037
%
% This function is being called in the main File Named: Fundamental_Matrix.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [newpts, T] = normalise(pts)

    % Find the indices of the points that are not at infinity
    pts = pts';
    index = find(abs(pts(3,:)) > eps);
    
    % For the finite points ensure homogeneous coordinates have scale of 1
    pts(1,index) = pts(1,index)./pts(3,index);
    pts(2,index) = pts(2,index)./pts(3,index);
    pts(3,index) = 1;
    
    c = mean(pts(1:2,index)')';           
    newp(1,index) = pts(1,index)-c(1);
    newp(2,index) = pts(2,index)-c(2);
    
    dist = sqrt(newp(1,index).^2 + newp(2,index).^2);
    meandist = mean(dist(:));
    
    scale = sqrt(2)/meandist;
    
    T = [scale   0   -scale*c(1)
         0     scale -scale*c(2)
         0       0      1      ];
    
    newpts = T*pts;
    
    
end
