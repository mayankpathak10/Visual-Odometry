%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 673 Perception For Autonomous Robotics
% PROJECT 2  - Spring 2018
%
% Visual Odometry 
% FUNCTION - Essential_Matrix
% 
% It Calculates Essential Matrix using Input arguments as Fundamental
% Matrix, Camera Intrinsic Matrix, Rotation Matrix and Translation Matrix.
% 
% The Function then calculate ROtation and Translation Matrix and returns
% the best rotation and translation matrix.
% 
% 
% Code By: Mayank Pathak
%          115555037
% 
% Dependencies: No Dependencies.
%
% This function is being called in the main File Named: Visual_Odometry.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [R,T] = Essential_Matrix(F,K,R,T)    
    E1 = K'*F*K;
    %  
    [U1,S1,V1] = svd(E1);

    W = [0 -1 0;1 0 0;0 0 1];
    Z = [0 1 0;-1 0 0; 0 0 0];
     
    S1(3,3)=0;

    E2 = U1*S1*V1';
    
    [U2,~,V2] = svd(E2);

    R1 = U2*W'*V2';
    R2 = U2*W*V2';

    if det(R1) < 0
        R1 = -R1;
    end

    if det(R2) < 0
        R2 = -R2;
    end 
    
    Tx = U2*Z*U2';

    t1 = [Tx(3,2) Tx(1,3) Tx(2,1)];  %same as taking last column of U2
    t2 = -t1;
    
    check=1;
    
    if t1(3) > 0
        T_temp = t1;
    
    elseif t2(3) > 0
        T_temp = t2;
    
    else
        check=0;
    end

    if(R1(1,1)>0 && R1(2,2)>0.9 && R1(3,3)>0)
       R_temp = R1;

    elseif (R2(1,1)>0 && R2(2,2) > 0.9 && R2(3,3) > 0)
       R_temp = R2;
    
    else
        check=0;
    end
    
    if check==0
        R_temp = eye(3);
        T_temp = zeros(1,3);
    end
    
    R = R_temp*R;
    T = T + T_temp*R;
    
%     Restricting the rotation to only about Y axis
%     R(1,2) = 0;
%     R(2,1) = 0;
%     R(2,3) = 0;
%     R(3,2) = 0;
    
%     Reducing Noise
    if abs(R(1,3))<0.001
        R(1,3) = 0;
    end
    if abs(R(3,1))<0.001
        R(3,1) = 0;
    end
    
%     if abs(T(1))<0 ||  R(1,1)>0.99
%         T = [0,0,T(1,3)];
%     end
end
 