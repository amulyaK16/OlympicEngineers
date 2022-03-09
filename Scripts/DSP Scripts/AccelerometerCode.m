%lab 6 and find some other code snippets of analyzing acceleration
%https://www.mathworks.com/help/supportpkg/arduinoio/ref/mpu6050-system-object.html
%https://www.mathworks.com/help/nav/ref/accelparams.html
function accel=myAccel(accelerometer)
a=load('Accel1.csv');
t1=0:2:((size(a,1)*2)-1);
accelX=a(:,[1]);
accelY=a(:,[2]);
accelZ=a(:,[3]);
gyroX=a(:,[4]);
gyroY=a(:,[5]);
gyroZ=a(:,[6]);
%Plot of acceleration in 3-axis with respect to time
figure(1);
subplot(3,1,1);
plot(t1,accelX);
title('Plot of acceleration in x-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,2);
plot(t1,accelY);
title('Plot of acceleration in y-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,3);
plot(t1,accelZ);
title('Plot of acceleration in z-direction');
xlabel('Time');
ylabel('Acceleration');

figure(2);
subplot(3,1,1);
plot(t1,gyroX);
title('Plot of Gyroscope data in x-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,2);
plot(t1,gyroY);
title('Plot of Gyroscope data in y-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,3);
plot(t1,gyroZ);
title('Plot of Gyroscope data in z-direction');
xlabel('Time');
ylabel('Acceleration');
data=load('Alltest1_finalrpt.csv');
t2=0:2:((size(data,1)*2)-1);
accelX1=data(:,[4]);
accelY1=data(:,[5]);
accelZ1=data(:,[6]);
gyroX1=data(:,[7]);
gyroY1=data(:,[8]);
gyroZ1=data(:,[9]);
%Plot of acceleration in 3-axis with respect to time
figure(3);
subplot(3,1,1);
plot(t2,accelX1);
title('Plot of acceleration in x-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,2);
plot(t2,accelY1);
title('Plot of acceleration in y-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,3);
plot(t2,accelZ1);
title('Plot of acceleration in z-direction');
xlabel('Time');
ylabel('Acceleration');

figure(4);
subplot(3,1,1);
plot(t2,gyroX1);
title('Plot of Gyroscope data in x-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,2);
plot(t2,gyroY1);
title('Plot of Gyroscope data in y-direction');
xlabel('Time');
ylabel('Acceleration');
subplot(3,1,3);
plot(t2,gyroZ1);
title('Plot of Gyroscope data in z-direction');
xlabel('Time');
ylabel('Acceleration');

end
