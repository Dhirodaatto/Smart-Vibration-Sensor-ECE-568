T = readtable("accel_capture.csv", "FileType","text", "CommentStyle","#");
t = T.t_us * 1e-6;
ax = T.ax_g;  ay = T.ay_g;  az = T.az_g;

Fs = 1000;                    % or Fs = 1/mean(diff(t));
x = ax - mean(ax);            % remove DC
N = numel(x);

X = fft(x);
f = (0:N-1)*(Fs/N);

% Single-sided magnitude
P2 = abs(X)/N;
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f1 = f(1:floor(N/2)+1);

plot(f1, P1); grid on;
xlabel("Hz"); ylabel("|FFT|");
title("Accel X FFT");