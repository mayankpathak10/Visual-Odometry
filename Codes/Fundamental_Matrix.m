%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 673 Perception For Autonomous Robotics
% PROJECT 2  - Spring 2018
%
% Visual Odometry 
% FUNCTION - Fundamental_Matrix
% 
% It Calculates the Fundamental Matrix on the basis of Given matching
% POints of both the images.
% 
% The Input points must be an array of size N x 3.
%
% 
% Code By: Mayank Pathak
%          115555037
% 
% Dependencies: This function calls another function named normalise.m
%
% This function is being called in the main File Named: RANSACfundamental_matrix.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ F_matrix ] = Fundamental_Matrix(Points_a,Points_b)

% Normalize the input points using the function normalise.
[Normalized_a, Ta] = normalise(Points_a);
[Normalized_b, Tb] = normalise(Points_b);

%  Defining A Matrix
A = [Normalized_a(1,:)'.*Normalized_b(1,:)'   Normalized_a(1,:)'.*Normalized_b(2,:)'  Normalized_a(1,:)' ...
    Normalized_a(2,:)'.*Normalized_b(1,:)'   Normalized_a(2,:)'.*Normalized_b(2,:)'  Normalized_a(2,:)' ...
    Normalized_b(1,:)'             Normalized_b(2,:)'  ones(length(Normalized_a),1)];

[~,~,V] = svd(A,0);
f = V(:,end);
F = reshape(f, [3,3])';

[U,S,V] = svd(F);
S(3,3) = 0;
F = U*S*V';
F = F/norm(F);
%Denormalizing and rescaling
F_matrix = Tb' * F * Ta;
% F_matrix = F_matrix/norm(F_matrix);
end