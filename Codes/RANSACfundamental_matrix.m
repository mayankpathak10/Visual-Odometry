%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 673 Perception For Autonomous Robotics
% PROJECT 2  - Spring 2018
%
% Visual Odometry 
% FUNCTION - RANSACfundamental_matrix
% 
% It implements RANSAC algorithm on the fundamemtal matri obtained through
% another function called within this function. 
% 
% With the matching points as inputs, the function creates fundamental
% matrix for each set of points and then gives out the best Fundamental
% matrix depending upon the number of inlier points.
%
% References: Code by Anish Gartia on stencil made by Henry Hu was referred
%             to understand the implementation of RANSAC.
% 
% Code By: Mayank Pathak
%          115555037
%
% Dependencies: The function Calls another function named
% Fundamental_Matrix.m.
% 
% This function is being called in the main File Named: Visual_Odometry.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [ Best_Fmatrix, inliers_a, inliers_b] = RANSACfundamental_matrix(matches_a, matches_b)

% Initialization
ptsPerItr = 10;
maxInliers = 0;
errThreshold = 0.02;
Iterations = 1000;

Best_Fmatrix = zeros(3,3);
xa = [matches_a ones(size(matches_a,1),1)];
xb = [matches_b ones(size(matches_b,1),1)];


% Loop to iterate through the available points
for i = 1:Iterations
    ind = randi(size(xa,1), [ptsPerItr,1]);
%  Calling the function to return a Fundamental matrix using given points
    FmatrixEstimate = Fundamental_Matrix(xa(ind,:), xb(ind,:));   
    err = sum((xb .* (FmatrixEstimate * xa')'),2);
    currentInliers = size( find(abs(err) <= errThreshold) , 1);
    if (currentInliers > maxInliers)
       Best_Fmatrix = FmatrixEstimate; 
       maxInliers = currentInliers;
    end    
end

err = sum((xb .* (Best_Fmatrix * xa')'),2);
[~,I]  = sort(abs(err),'ascend');

%  Determining Inlier points
inliers_a = matches_a(I(1:min(30,size(I))),:);
inliers_b = matches_b(I(1:min(30,size(I))),:);

end