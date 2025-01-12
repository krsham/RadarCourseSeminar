clc;clear;close all;




c = 3e8;   % Propagation speed


range_res = 50;         % Required range resolution
pulse_bw = 10e6;    % Pulse bandwidth
pw = 0.00001;               % Pulse width
fs = 10*pulse_bw;              % Sampling rate
fc = 60E9;
lambda = c/fc;

freq = phased.FMCWWaveform('SweepBandwidth',pulse_bw,'SampleRate',fs,'SweepTime',pw);
fmWave = freq.step;

fmWave = frequencyOffset(fmWave,fs,-pulse_bw/2);

figure;plot(real(fmWave)); hold; plot(imag(fmWave)); hold
title("LFM Pulse for FFT matched filter")

mf = conj(fmWave);

