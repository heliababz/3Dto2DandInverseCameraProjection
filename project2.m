%% Initialize Video Reader 
    videoFile = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
    vue2Video = VideoReader(videoFile);
    videoFile = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
    vue4Video = VideoReader(videoFile);
    mocapFnum = 808;
    
    vue2Video.currentTime = (mocapFnum-1)*(50/100)/vue2Video.FrameRate;
    vid2Frame = readFrame(vue2Video);
    
    vue4Video.currentTime = (mocapFnum-1)*(50/100)/vue4Video.FrameRate;
    vid4Frame = readFrame(vue4Video);
     
    
%%Load Mocap data sets
    load('Subject4-Session3-Take4_mocapJoints.mat');
    size = 26214;

counter = 1;
for i = 1:size
    conf = mocapJoints(i,:,4);
     
    if conf == 1
        frame(counter).joints = mocapJoints(i,:,1);
        frame(counter).y = mocapJoints(i,:,2);
        frame(counter).z = mocapJoints(i,:,3);
        counter = counter + 1;
    end
end
     
%% Load camera parameters 
    load('vue2CalibInfo.mat');
    load('vue4CalibInfo.mat');
    %Find M matrix using extrinsic (rotation and translation) and
    % instrisic (affine and projection) parameters
    
    [M2, location2] = Calculate_M_Matrix(vue2)
    [M4, location4] = Calculate_M_Matrix(vue4)
    
%% Use cameera parameters to project 3D joints into each of the two image coordinate systems

xyzm = [frame(mocapFnum).joints; frame(mocapFnum).y; frame(mocapFnum).z; ones(1,12)];

coordinate2 = M2*xyzm   
coordinate2 = coordinate2./coordinate2(3,:); %this will be my left image used in triangulation

coordinate4 = M4*xyzm
coordinate4 = coordinate4./coordinate4(3,:); %this will be my right image used in triangulation

figure;
image(vid2Frame);
axis on 
hold on;
for i=1:12
    plot(coordinate2(1,i), coordinate2(2,i), '*','MarkerSize',3, 'MarkerEdgeColor','r')
end
hold off 

figure;
image(vid4Frame);
axis on 
hold on;
for i=1:12
    plot(coordinate4(1,i), coordinate4(2,i), '*','MarkerSize',3, 'MarkerEdgeColor','r')
end
hold off


%%Reconstruct the 3D location of the 2D joints using two-camera triangulation 

reconstructed3Dpoints = triangulate(coordinate2, coordinate4, M2, M4);

%% Error

%Euclidean distance 

L2 = Euclidean(xyzm, reconstructed3Dpoints)
mean = mean(L2)
std = std(L2)
min = min(L2)
max = max(L2)
median = median(L2)
%counter is of size of the frames with conf==1

%{
for i=1:counter
    original = [frame(i).joints; frame(i).y; frame(i).z; ones(1,12)];
    twoDim2 = M2*original; 
    twoDim4 = M4*original; 
    recon = triangulate(twoDim2, twoDim4, M2, M4);
    D2 = Euclidean(original, recon);
    
    frame(i).stats = zeros(1,12);
    
    frame(i).stats(1) = mean(D2);
    frame(i).stats(2) = std(D2);
    frame(i).stats(3) = min(D2);
    frame(i).stats(4) = max(D2);
    frame(i).stats(5) = median(D2);
end
%}

%% Epipolar lines
%left epipole is the projection of the right camera (O') on the image plane
left_epipole = M2*location4;
left_epipole = left_epipole/left_epipole(3)


%right epipole is the projection of the left camera (O) on the image plane
right_epipole = M4*location2;
right_epipole = right_epipole/right_epipole(3)


left_epipolar_lines = zeros(2,12);  % each col is [m; b;]
right_epipolar_lines = zeros(2,12); % each col is [m; b;]
for i=1:12
    x = [left_epipole(1) coordinate2(1,i)];
    y = [left_epipole(2) coordinate2(2,i)];
    
    c = [[1; 1] x(:)]\y(:);
    %slope = c(2);
    %intercept = c(1);
    left_epipolar_lines (:,i) = [c(2); c(1)];
    
    x = [right_epipole(1) coordinate4(1, i)];
    y = [right_epipole(2) coordinate4(2, i)];
    
    c = [[1; 1] x(:)]\y(:);
    %slope = c(2);
    %intercept = c(1);
    right_epipolar_lines(:,i) = [c(2); c(1)];
end

%{
THIS CHUNK OF CODE DOESN'T WORK, MY FUNDAMENTAL MATRIX IS WRONG
%Get Fundamental Matrix 
F = Fundamental_Matrix(coordinate2, coordinate4);
%check whether this is correct x'TFx = 0
coordinate4(:,3)' * F * coordinate2(:,3) % --> this is failing so my F is wrong

%solve for the right epipolar line: right_epipolar_lines  
% --->> l = Fx (where x is in the left image)

right_epipolar_lines = F*coordinate2;

%solve for the left epipolar line: left_epipolar_lines 
% --->> l' = FTx' (where x' is in the right image)

left_epipolar_lines = F'*coordinate4;
%}


%% plot the epipolar lines in the left image

figure;
image(vid2Frame);
hold on
plot(left_epipole(1), left_epipole(2),'*','MarkerSize',3, 'MarkerEdgeColor','b')

for i=1:12
    line = left_epipolar_lines(:,i);
    m = line(1);
    b = line(2);
    x = [1, 2000];

    y = m*x+b;
    
    plot(coordinate2(1,i), coordinate2(2,i), '*','MarkerSize',3, 'MarkerEdgeColor','r')
    plot(x, y,'MarkerSize',3, 'MarkerEdgeColor','b')
end
hold off 

%% plot the epipolar lines in the right image
figure;
image(vid4Frame);
hold on
plot(right_epipole(1), right_epipole(2),'*','MarkerSize',3, 'MarkerEdgeColor','b')

for i=1:12
    line = right_epipolar_lines(:,i);
    m = line(1);
    b = line(2);
    x = [1, 2000];
    %move around ax + by + c = 0
    y = m*x+b;
    
    plot(coordinate4(1,i), coordinate4(2,i), '*','MarkerSize',3, 'MarkerEdgeColor','r')
    plot(x, y,'MarkerSize',3, 'MarkerEdgeColor','b')
end
hold off 






