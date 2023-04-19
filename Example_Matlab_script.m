% Clear the workspace (delete all variables)
clear all
% Clean up the command window
clc

% Let's use PUMA 560 robot as an example 
% Various robot models are located at rvctools\robot\models
% Standard D-H representation
mdl_puma560
% Modified D-H representation
mdl_puma560akb

% Define the position of the canvas (relative to the robot)
% The position vector from the origin of the robot frame FR to the origin 
% of the canvas frame FC 
p = [-0.4; 0.4; 0.14]
% Define the orientation of the canvas (relative to the robot)
% The orientation is represented using a 3x3 rotation matrix
% Assume the canvas is tilted 45 degrees about the x axis of FR
R = rotx(degtorad(45))
% Another way to create the same rotation matrix
R = angvec2r(degtorad(45), [1; 0; 0])
% Construct the transformation matrix from FC to FR
T = [R p]
T = [T; [0 0 0 1]]

% Define/design the letters to be written on the canvas, namely, define the 
% the path to write teh letters by specifying points (in FC)
% Define/design the letter Z using 4 points
% Note the last/4th element is always 1 (the homogeneous coordinate)
A = zeros(4, 10)
A(:,1)  = [ 0.2;     0.1;    0;  1]
A(:,2)  = [ 0.4;     0.1;    0;  1]
A(:,3)  = [ 0.2;    -0.1;    0;  1]
A(:,4)  = [ 0.4;    -0.1;    0;  1]
% Define/design the letter S using 6 points
A(:,5)  = [ 0.1;     0.1;    0;  1]
A(:,6)  = [ 0.0;     0.2;    0;  1]
A(:,7)  = [-0.1;     0.1;    0;  1]
A(:,8)  = [ 0.1;    -0.1;    0;  1]
A(:,9)  = [ 0.0;    -0.2;    0;  1]
A(:,10) = [-0.1;    -0.1;    0;  1]

% Bring up a figure window
figure(1)
% Clean up the figure
clf;
% Plot the defined/designed letters Z and S on canvas (i.e., in FC)
plot(A(1,1:4), A(2,1:4), '-rx', A(1,5:10), A(2,5:10), '-rx')
% Setup axis ranges
xlim([-1,1]);
ylim([-1,1]);
% Label axes
xlabel('X axis of canvas frame FC')
ylabel('Y axis of canvas frame FC')
% Turn on grid
grid on
% Specify a title
title('Letters to be writen on canvas')

% Convert all points from canvas frame FC to robot frame FR
B = zeros(4, 10)
B = T*A

% Construct transformation matrices representing the points in FR
% You can control the orientation of the end-effector by constructing
% particular rotation matrices. 
TB = zeros(4, 4, 10)
% one way to specify the rotation maxtrix representing the orientation of
% the end-effector
Rr = [sqrt(2)/2 0 sqrt(2)/2 ;0 1 0;-sqrt(2)/2 0 sqrt(2)/2;0 0 0]
% A second way to specify the rotation maxtrix representing the orientation 
% of the end-effector
Rr = [1 0 0; 0 1 0; 0 0 1; 0 0 0]
% A third way to specify the rotation maxtrix representing the orientation 
% of the end-effector
Rr = [rotz(degtorad(90))*roty(degtorad(90)); 0 0 0]

% Replace the "orientation area" of the transformation matrices with the 
% above contructed rotation matrix. For the follwing example, the robot 
% writes both letters with the same orientation. You can change it!
for i = 1:1:10
    TB(:,:,i) = [Rr B(:,i)]
end

% Trajectory planning in Cartesian space. One can insert more points 
% between two adjacent points in matrix B. 
n = 4
TP = zeros(4, 4, n*10)
% For points defining letter Z, insert n points between two adjacent points
for i = 1:1:3
    TP(:,:,((i-1)*n+1):i*n) = ctraj(TB(:,:,i), TB(:,:,i+1), n)
end
% For points defining letter S, insert n points between two adjacent points
for i = 5:1:9
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

% "Write" the letters. Plot the points while moving the robot so that it 
% looks the robot is writing the letters.
% "Write" the letter Z
for i = 1:1:(n*4)
    plot2(transpose(TP(1:3,4,i)),'r.')
    p560.plot(q(i,:))   
    hold on 
end
% "Write" the letter S
for i = (n*4):1:(n*10)
    plot2(transpose(TP(1:3,4,i)),'r.')
    p560.plot(q(i,:))   
    hold on 
end

% Move robot to the vertical "ready" or "home" configuration after finish
% writing
p560.plot(qr)  