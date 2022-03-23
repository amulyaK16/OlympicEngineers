function [freq, ampl, BPM] = matlab_functions(ecg_t)

    X = 1:length(ecg_t);
    Y = ecg_t;

    Fs = 500;            % Sampling frequency 
    L = length(X);        % Length of signal
    yf = fft(Y);
    P2 = abs(yf/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    [~,loc] = findpeaks(P1, f, 'SortStr', 'descend', 'NPeaks', 2);

    freq = f;
    ampl = P1;
    BPM = loc(1)*60;
end