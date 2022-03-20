function [EMG] = EMGCode(EMGtest)
%data=load('EMGlog1_finalrpt.csv');
%b=data(:,[2]);
b = EMGtest;

t1=0:2:((size(b,1)*2)-1);
figure(1);
plot(t1,b);
title('Plot of Raw EMG data');
fs=800;
fny=fs/2;
y=abs(b-mean(b));
fco=20;
[b,a]=butter(2,fco*1.25/fny);
z=filtfilt(b,a,y);
figure(2);
plot(t1,y-4,'b',t1,y,'g',t1,z,'r');
xlabel('Time (s)'); ylabel('EMG (V)');
legend('Raw (offset)','Rectified','Linear envelope');

% a=load('EMGtest1log.txt');
% t=0:2:((size(a,1)*2)-1);
% figure(2);
% plot(t,a);
% title('Plot of filtered output for EMG data using reference electrode');
%The data is amplified, rectified, and integrated
%This output is just EMG noise because this 1 electrode that is being used
%is the reference electrode and must be placed on a boney area close to the
%target zone. 
%Additional tools are required to remove excess length of pins to be able
%to use the electrodes on the sensor

%%LINKS:%%
%https://www.mathworks.com/help/matlab/ref/cumtrapz.html
%Lab 2 of SYSC 4203
%Calculate the RMS
% movRMS=dsp.MovingRMS(2000);
% y=movRMS(b);
% 
% figure(3);
% plot(y);
% 
% %%Change the angle of movement%%
% % %Change from 120 degrees to 90 degrees
% x1=120:-30/(100-1):90;
% x1=x1';
% y1 =y(1:100);
% %Change from 90 degrees to 60 degrees
% x2=90:-30/(500-101):60;
% x2=x2';
% y2 =y(101:500);
% %Change from 60 degrees to 30 degrees
% x3=60:-30/(700-501):30;
% x3=x3';
% y3 =y(501:700);
% figure(10);
% xTotalsub2 = [x1;x2;x3];
% yTotalsub2 = [y1;y2;y3];
% plot(xTotalsub2,yTotalsub2)

end

