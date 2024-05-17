% Potential field Script

clear all 
close all

%% Generate points

nrows = 400;
ncols = 600;

obstacle = false(nrows,ncols);

[x, y] = meshgrid(1:ncols, 1:nrows);
%here note that x corresponds to cols while the y corresponds to rows

%% Generate some obstacles 
% from Circular_obstacle_3D
obstacle(300:end, 100:250) = true; % rectangular obstacle
obstacle(150:200, 400:500) = true; % rectangular obstacle 

t = ((x-200).^2 + (y-50).^2 < 50^2) ; %circular obstacle
obstacle(t) = true;
t = ((x-400).^2 + (y-300).^2 < 100^2) ; %circular obstacle
obstacle(t) = true; %map every point where the obstacle lies as true.

%% Now compute the distance trasform 
%from DistanceFromObstacle script and the scaling factor script

d = bwdist(obstacle); %distance transform assigns a number that is the 
% distance between that pixel and the nearest nonzero pixel of BW.
%bwdist is the matlab function which returns distance from any true element
%in ostacle way

%% Compute the Repulsive force.
K = 100;
Rho = d/K +1;
%Note some values in d might be 0 will cause problems in calculating the
%repulsive force so we add a addition zero to aviod the division by zero.

d0 = 2; %if any robot away form the obstacle by do unit its repulsive force
%is considered zero

Eta = 1000;
%Used to control repulsive force large Eta will cause some balance
%between repulsive and attractive forces we need to make large repulsive
%force so that it dosent gets struck on obstacles.

repulsive = (Eta/2)*((1./Rho-1/d0).^2);

repulsive(Rho > d0) = 0;
max(max(repulsive))

%% Display the repulsive potential

figure;
m = mesh(repulsive);
m.FaceLighting = "phong";
axis([-10 600 -10 400])

title ("Repulsive Potential")

%%  Compute the Actractive force.

goal = [400, 50];

zeta = 1/700 ;
% used to control the strength of attractivness towards goal if zeta =
% 1/10000, the robot will not reach the location but it 1/10 it will cross
% over the obstacle need to take an optimium solution.

attractive = zeta * ( (x- goal(1)).^2 + (y-goal(2)).^2);

figure;
m = mesh(attractive);
m.FaceLighting = 'phong';
axis equal;

title ('Attractive Potential');
max(max(attractive))

% %% Display 
% 
% figure;
% imshow(~obstacle);
% 
% hold on
% plot(goal);
% hold off
% 
% axis([0 ncols nrows]);
% axis xy;
% axis on;
% 
% xlabel('x');
% ylabel('y');
% 
% title('Working space');

%% Combinning both the potentials

f = attractive + repulsive ;

figure;
m = mesh(f);
% m.FaceLighting('phong');
axis equal;

title("Total Potential")

%% Plan route 
start = [50, 50];

route = GradientBasedPlanner(f, start, goal, 1000);

%% Plot the energ surface.

figure;
m = mesh(f);
axis equal

%% Plot a ball to vishualize 

[sx, sy, sz] = sphere();

R = 10;
sx = R*sx;
sy = R*sy;
sz = R*sz +R;
% the lower half will not be vissible if added R

hold on;
p = mesh(sx,sy,sz);
%this will plot the ball at 0,0,0
p.FaceColor = 'red';
p.EdgeColor = 'none';
% p.FaceLighting = 'phong';
hold off;

hold on
plot(goal(1),goal(2),'g*','MarkerSize',25);
hold off

%Plot the ball at each point in route from start to goal
for i = 1:size(route,1)
    P = round(route(i,:));
    %P = [x,y]
    z = f(P(2),P(1)); 
    % z = f(x,y)

    %Draw the ball shifted to the new pos
    p.XData = sx + P(1);
    p.YData = sy + P(2);
    p.ZData = sz + f(P(2),P(1));

    drawnow;

    pause(0.05)
end

%% Quiver plot 
[gx, gy] = gradient(-f);
skip = 20 ;
figure ;

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver(x(yidx,xidx),y(yidx,xidx),gx(yidx,xidx),gy(yidx,xidx),0.4);

axis([1 ncols 1 nrows]);

hold on ;

ps = plot(start(1), start(2), 'r.','MarkerSize',30);
pg = plot(goal(1),goal(2),'g.',MarkerSize=30);
pp3 = plot(route(:,1),route(:,2),'r','LineWidth',2);


