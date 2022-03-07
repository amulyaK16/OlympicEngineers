function ECG= myECG(pulseLog)
data=load('Alltest1_finalrpt.csv');
x1=data(:,[3]);
%heart rate variability 
%https://www.mathworks.com/matlabcentral/answers/40805-heart-rate-variability

% %%Pan Tompkin algorithm%%
fs=200;
N = length (x1);       % Signal length
t =[0:N-1]/fs;        % time index 
figure(1)
plot(t,x1)
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal')
figure(2)
subplot(2,1,1)
plot(t,x1)
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal')
subplot(2,1,2)
plot(t(200:1000),x1(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Canceling out DC drift and normalization
x1 = x1 - mean (x1 );    % cancel DC conponents
x1 = x1/ max( abs(x1 )); % normalize to one
 
figure(3)
subplot(2,1,1)
plot(t,x1)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after DC canceling and normalization')
subplot(2,1,2)
plot(t(200:1000),x1(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Lowpass filteration
b=[1 0 0 0 0 0 -2 0 0 0 0 0 1];
a=[1 -2 1];
 
 
h_LP=filter(b,a,[1 zeros(1,12)]); % transfer function of LPF
 
x2 = conv (x1 ,h_LP);
x2 = x2/ max( abs(x2 )); % normalize , for convenience .
 
figure(4)
subplot(2,1,1)
plot([0:length(x2)-1]/fs,x2)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after Lowpass filtering')
xlim([0 max(t)])
subplot(2,1,2)
plot(t(200:1000),x2(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Highpass filteration
b = [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 32 -32 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1];
a = [1 -1];
 
h_HP=filter(b,a,[1 zeros(1,32)]); % impulse response iof HPF
 
x3 = conv (x2 ,h_HP);
x3 = x3/ max( abs(x3 ));
 
figure(5)
subplot(2,1,1)
plot([0:length(x3)-1]/fs,x3)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after Highpass filtering')
xlim([0 max(t)])
subplot(2,1,2)
plot(t(200:1000),x3(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Derivative filteration
% Make impulse response
h = [-1 -2 0 2 1]/8;
% Apply filter
x4 = conv (x3 ,h);
x4 = x4 (2+[1: N]);
x4 = x4/ max( abs(x4 ));
 
figure(6)
subplot(2,1,1)
plot([0:length(x4)-1]/fs,x4)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after Derivative Filtration')
subplot(2,1,2)
plot(t(200:1000),x4(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Squaring
x5 = x4 .^2;
x5 = x5/ max( abs(x5 ));
figure(7)
subplot(2,1,1)
plot([0:length(x5)-1]/fs,x5)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after Squaring')
subplot(2,1,2)
plot(t(200:1000),x5(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

%Moving window Integration
% Make impulse response
h = ones (1 ,31)/31;
Delay = 15; % Delay in samples
 
% Apply filter
x6 = conv (x5 ,h);
x6 = x6 (15+[1: N]);
x6 = x6/ max( abs(x6 ));
 
figure(8)
subplot(2,1,1)
plot([0:length(x6)-1]/fs,x6)
xlabel('Time');ylabel('Amplitude (mV)');title(' ECG Signal after Averaging')
subplot(2,1,2)
plot(t(200:1000),x6(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
xlim([1 3])

figure(9)
plot(t(200:1000),x6(200:1000))
xlabel('Time');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
end 

% %Finding the QRS points
% figure(7)
% subplot(2,1,1)
% plot([0:length(x6)-1]/fs,x6)
% xlabel('second');ylabel('Amplitude (mV)');title(' ECG Signal after Averaging')
% subplot(2,1,2)
% plot(t(200:1000),x6(200:1000))
% xlabel('second');ylabel('Amplitude (mV)');title('Input ECG Signal within a window')
% xlim([1 3])
% figure(7)
% subplot(2,1,1)
% max_h = max(x6);
% thresh = mean (x6 );
% P_G= (x6>0.01);
% difsig=diff(P_G); 
% figure (8)
% subplot(2,1,1)
% hold on
% plot (t(200:1000),x1(200:1000)/max(x1))
% box on
% xlabel('second');ylabel('Integrated')
% xlim([1 3])
% subplot(2,1,2)
% plot (t(200:1000),x6(200:1000)/max(x6))
% xlabel('second');ylabel('Integrated')
% xlim([1 3])
%  
% left=find(difsig==1); 
% raight=find(difsig==-1);
% left=left-(6+16); 
% raight=raight-(6+16);
% 
% for i=1:length(left)-1
%    [R_value(i) R_loc(i)] = max( x1(left(i):raight(i)) );
%     R_loc(i) = R_loc(i)-1+left(i); % add offset
%  
%     [Q_value(i) Q_loc(i)] = min( x1(left(i):R_loc(i)) );
%     Q_loc(i) = Q_loc(i)-1+left(i); % add offset
%  
%     [S_value(i) S_loc(i)] = min( x1(left(i):raight(i)) );
%     S_loc(i) = S_loc(i)-1+left(i); % add offset
%     
% 
% end
% Q_loc=Q_loc(find(Q_loc~=0));
% R_loc=R_loc(find(R_loc~=0));
% S_loc=S_loc(find(S_loc~=0));
% 
% figure
% subplot(2,1,1)
% title('ECG Signal with R points');
% plot (t,x1/max(x1) , t(R_loc) ,R_value , 'r^', t(S_loc) ,S_value, '*',t(Q_loc) , Q_value, 'o');
% legend('ECG','R','S','Q');
% subplot(2,1,2)
% plot (t,x1/max(x1) , t(R_loc) ,R_value , 'r^', t(S_loc) ,S_value, '*',t(Q_loc) , Q_value, 'o');
% xlim([1 3])
