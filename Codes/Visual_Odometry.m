%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 673 Perception For Autonomous Robotics
% PROJECT 2  - Spring 2018
%
% Question ( 100 points)- Visual Odometry 
%
% To compare frames obtained from a dashboard camera and give the movement
% of camera center in the form of line shown in a new image.
%
% 
% 
% Code By: Mayank Pathak
%          115555037
%
% Dependencies: This code uses three functions,
%               
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Importing Images from the input folder
images = imageDatastore('..\Inputs\stereo\centre\*.png');  
nfiles = size(images.Files,1);    % Number of files found

% Importing Camera Model and setting Camera Paramenters
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel('..\Inputs\stereo\centre' ,'..\Inputs\model'); 
K = [fx 0 cx;0 fy cy;0 0 1];        % Variable to store Intrinsic Matrix
cameraParams = cameraParameters('IntrinsicMatrix',K);  

% Initialilzing Variables
cameraCenterPlot_In = [0,0];
cameraCenterPlot_my = [0,0];
R_In = eye(3);
T_In = zeros(1,3);
  
R_my = eye(3);
T_my = zeros(1,3);

figure;
Start_Frame = 1;

% Video Writer Object
Out_Video = VideoWriter('..\Output\Project2_Video.avi');
Out_Video.FrameRate = 100;
open(Out_Video);

%  Loop to iterate through frames and apply operations
for k = Start_Frame:1:(nfiles-1)
% disp(k);
% To read pair of images from the directory
Image1 = readimage(images,k);
Image2 = readimage(images,k+1);

% Demosaic to obtain color images from the Bayer Encoded Image
Image1=demosaic(Image1,'gbrg');
Image2=demosaic(Image2,'gbrg');
    
% Undistorting the Images using the given UndistortImage function
Undistorted_Image1 = UndistortImage(Image1, LUT);
Undistorted_Image2 = UndistortImage(Image2, LUT);

% Convert the images to gray scale for detecting features
First = rgb2gray(Undistorted_Image1);
Second = rgb2gray(Undistorted_Image2);

% detecting SURF features on both the images
Image1_points = detectSURFFeatures(First);
Image2_points = detectSURFFeatures(Second);

% extract features from images
[features1,vpoints_1] = extractFeatures(First,Image1_points);
[features2,vpoints_2] = extractFeatures(Second,Image2_points);

% Matching the extracted features
indexPairs = matchFeatures(features1,features2) ;
matchedPoints1 = vpoints_1(indexPairs(:,1));
matchedPoints2 = vpoints_2(indexPairs(:,2));
    
% A condition to implement search only if more than eight matching points 
% are found
if size(matchedPoints1.Location,1)<8

else
%   Function to calculate Fundamental Matrix Using RANSAC
    [Fmatrix, inliers_a, inliers_b] = RANSACfundamental_matrix(matchedPoints1.Location,matchedPoints2.Location);
    
%   In-Built Function to obtain Fundamental Matrix 
    F_matlab = estimateFundamentalMatrix(matchedPoints1,matchedPoints2);
    [FU, FD, FV] = svd(F_matlab);
    FD(3,3)=0;
    F = FU*FD*FV';  
    
%  Function to Calculate Essential
   [R_In,T_In] = Essential_Matrix(F,K,R_In,T_In);
   [R_my,T_my] = Essential_Matrix(Fmatrix,K,R_my,T_my);
   
   cameraCenterPlot_In(k,:)=[T_In(1,1) T_In(1,3)];
   cameraCenterPlot_my(k,:)=[T_my(1,1) T_my(1,3)];
 
%  Section to Plot results on the figure
 drawnow
 plot(cameraCenterPlot_In(:,1),cameraCenterPlot_In(:,2),cameraCenterPlot_my(:,1),cameraCenterPlot_my(:,2),'LineWidth',2);
    lowestx = min(T_In(1,1),T_my(1,1));
    lowesty = min(T_In(1,3),T_my(1,3));
    maximumx = max(T_In(1,1),T_my(1,1));
    maximumy = max(T_In(1,3),T_my(1,3));
    axis([min(-200,lowestx) max(1500,maximumx) min(-300,lowesty) max(1500,maximumy)]);
    legend('Inbult','My','Location','northwest');
    title(['Frame ',num2str(Start_Frame),' to ',num2str(k+1)]);
    frame = getframe;
    img = frame2im(frame);
    writeVideo(Out_Video,img);
end
end

close(Out_Video);

