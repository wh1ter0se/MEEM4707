clc, clear
close all

%% Define geom
x0 = 0.5*cos(deg2rad(30));
y0 = 0.5*sin(deg2rad(30));
r = 0.3;
theta = [deg2rad(30-90):0.001:deg2rad(30+90)];
d_res = 0.02;

%% Calculate points
points = [];
for phi = theta
    x1 = x0 + (0.3*cos(phi));
    y1 = y0 + (0.3*sin(phi));
    points = [points; snap(x1, d_res), snap(y1, d_res)];
end
unique(points,'rows')

%% Plot
scatter(points(:,1), points(:,2), 'b'), hold on;
scatter(x0, y0, 'r');
grid on;
xlim([-1,1]);
ylim([-1,1]);
xlabel('X (m)');
ylabel('Y (m)');
title('Possible Center Points');

%% Util
function rounded = snap(original, res)
    rounded = res*round(original/res)
end