function loadCell=loadCellAmp(loadcellData)
data=load('Alltest1_finalrpt.csv');
loadCellData=data(:,[1]);
t1=0:2:((size(data,1)*2)-1);
t1=t1';
figure(1);
plot(t1,loadCellData);
title('Load cell amplifier data over time');
xlabel('time');
ylabel('Load cell output (lbs)');
%Sources:
%https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide
%https://www.sparkfun.com/products/13879
end
