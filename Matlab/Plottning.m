%% Read file
filename = 'LOG_OUTPUT.txt';
delimiterIn = ',';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);

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

legend('Önskad styrvinkel', 'Verklig styrvinkel', 'Acceleration');
xlabel('tid [s]'); axis([0 max(t) 0 255]); grid on;
title('Loggad data');