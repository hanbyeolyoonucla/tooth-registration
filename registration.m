clc; clear all; close all;

% initial alignment
theta_x = pi/2;
theta_z = pi;
Rx = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
Rz = [cos(theta_z) -sin(theta_z) 0;sin(theta_z) cos(theta_z) 0;0 0 1];

% read scanned model
scan = stlread('./scan/tooth_surface.stl');
scanPoints = scan.Points;
scanPoints = scanPoints - mean(scanPoints,1);
scanPoints = (Rz*Rx*scanPoints')';
scanPoints = pointCloud(scanPoints);
% gridStep = 0.5;
% scanPoints = pcdownsample(scanPoints,'gridAverage',gridStep);

% read measured model
measure = readtable('./cmm/needle_valley_2.txt');
measurePoints = table2array(measure);
measurePoints = measurePoints - mean(measurePoints,1);
measurePoints = pointCloud(measurePoints);

[tformicp, movingRegicp, rmseicp] = pcregistericp(measurePoints,scanPoints);
[tformcpd, movingRegcpd, rmsecpd] = pcregistercpd(measurePoints,scanPoints);
[tformndt, movingRegndt, rmsendt] = pcregisterndt(measurePoints,scanPoints,1);
%%
figure(1)
pcshow(scanPoints.Location, 'g','MarkerSize', 10);
title('scan model')
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(2)
pcshow(measurePoints.Location,'r','MarkerSize', 100);
title('measure model')
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(3)
pcshow(movingRegicp.Location,'r','MarkerSize', 100);
hold on
pcshow(scanPoints.Location, 'g')
title('registration ICP')
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(4)
pcshow(movingRegcpd.Location,'r','MarkerSize', 100);
hold on
pcshow(scanPoints.Location, 'g')
title('registration CPD')
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(5)
pcshow(movingRegndt.Location,'r','MarkerSize', 100);
hold on
pcshow(scanPoints.Location, 'g')
title('registration NDT')
xlabel('X');
ylabel('Y');
zlabel('Z');