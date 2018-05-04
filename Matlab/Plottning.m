%% Read file
clear;
filename = 'LOG11.TXT';
delimiterIn = ',';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);

% Remove nevative values
%A.data(A.data(:,3) < -100, :) = [];

t =             A.data(:,1)./1000;
steeringRef =   A.data(:,2);
sensorValue =   A.data(:,3);
accelerate =    A.data(:,4);


%% Plot
clf;

plot(t,steeringRef, '--o', 'markersize', 3);
hold on;
plot(t,sensorValue, '--o', 'markersize', 3);
plot(t,accelerate, '--o', 'markersize', 3);
plot([min(t) max(t)], [128 128], 'k');

legend('�nskad styrvinkel', 'Verklig styrvinkel');
xlabel('tid [s]'); 
axis([min(t) max(t) 0 255]);
grid on;
title('Loggad data');