% Clear the workspace (delete all variables)
clear all
% Clean up the command window
clc

% Standard D-H representation
mdl_puma560
% Modified D-H representation
mdl_puma560akb

% Define the position of the counter (relative to the robot)
% The position vector from the origin of the robot frame FR to the origin 
% of the counter frame FC
p = [0.381; -0.1524; 0.6604];
%Rotation of Fc wrt Fr
R = [1 0 0; 0 1 0; 0 0 1];

%Transformation matrix of Fc wrt Fr
T = [R p];
T = [T; [0 0 0 1]]

% Path to cut the onion in Fc frame
A = zeros(4, 20);
%1st cut
A(:, 1) = [0.0381; 0.127; 0.0508; 1];
A(:, 2) = [0.0381; 0.127; 0; 1];
A(:, 3) = [0.1143; 0.127; 0; 1];
A(:, 4) = [0.1143; 0.127; 0.0508; 1];
%2nd cut
A(:, 5) = [0.0381; 0.1397; 0.0508; 1];
A(:, 6) = [0.0381; 0.1397; 0; 1];
A(:, 7) = [0.1143; 0.1397; 0; 1];
A(:, 8) = [0.1143; 0.1397; 0.0508; 1];
%3rd cut
A(:, 9) = [0.0381; 0.1524; 0.0508; 1];
A(:, 10) = [0.0381; 0.1524; 0; 1];
A(:, 11) = [0.1143; 0.1524; 0; 1];
A(:, 12) = [0.1143; 0.1524; 0.0508; 1];
%4th cut
A(:, 13) = [0.0381; 0.1651; 0.0508; 1];
A(:, 14) = [0.0381; 0.1651; 0; 1];
A(:, 15) = [0.1143; 0.1651; 0; 1];
A(:, 16) = [0.1143; 0.1651; 0.0508; 1];
%5th cut
A(:, 17) = [0.0381; 0.1778; 0.0508; 1];
A(:, 18) = [0.0381; 0.1778; 0; 1];
A(:, 19) = [0.1143; 0.1778; 0; 1];
A(:, 20) = [0.1143; 0.1778; 0.0508; 1];

A

%Plot the trajectory
figure(1);
clf;
plot3(A(1, :), A(2, :),  A(3, :), '-rx')

%Convert points from counter's reference frame to robot reference frame
B = zeros(4, 20);
B = T*A

% Construct transformation matrices representing the points in FR
% We can control the orientation of the end-effector by constructing
% particular rotation matrices.
TB = zeros(4, 4, 20);

% the rotation maxtrix representing the orientation 
% of the end-effector
Rr = [1 0 0; 0 1 0; 0 0 1; 0 0 0];

% Replace the "orientation area" of the transformation matrices with the 
% above contructed rotation matrix.
for i = 1:1:20
    TB(:,:,i) = [Rr B(:,i)];
end

TB

% Trajectory planning in Cartesian space.
n = 4;
TP = zeros(4, 4, n*20);
% For points defining the path, insert n points between two adjacent points
for i = 1:1:19
    TP(:,:,((i-1)*n+1):i*n) = ctraj(TB(:,:,i), TB(:,:,i+1), n)
end

% Solve the inverse kinematics problem to obtain the sets of joint 
% coordinates corresponding to the points defined in Cartesian space
q = p560.ikine(TP);

% Bring up a figure window, clean up, setup viewpoint and axis ranges
figure(2)
clf;
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);
view([105,14]);

% Plot the points while moving the robot so that it 
% looks the robot is cutting a vegetable
for i = 1:1:(n*20)
    plot2(transpose(TP(1:3,4,i)),'r.')
    p560.plot(q(i,:))   
    hold on 
end

% Move robot to the vertical "ready" or "home" configuration after finish
% writing
p560.plot(qr)
