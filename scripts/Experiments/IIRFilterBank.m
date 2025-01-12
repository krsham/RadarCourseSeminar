clc;clear; close all;



c = 3e8;   % Propagation speed


range_res = 50;         % Required range resolution
pulse_bw = 10e6;    % Pulse bandwidth
pw = 0.0001;               % Pulse width
fs = 10*pulse_bw;              % Sampling rate
fc = 60E9;
lambda = c/fc;

pri = pw* 10;

simLen = 100;

simSteps = simLen/pri;

lpfIIR = dsp.LowpassFilter("FilterOrder",6,"PassbandFrequency",fs/20,"SampleRate",fs,"StopbandAttenuation",60,"DesignForMinimumOrder",false,"FilterType","IIR");

% fmcwWave = frequencyOffset(waveForm.step,fs,-pulse_bw/2);
fmcwWave = ones(pw*fs,1);

tx_pulse = zeros(pri*fs,1);
tx_pulse(1:numel(fmcwWave),1) = fmcwWave*10e6;



rxPulse1 =  frequencyOffset(fmcwWave,fs,fs/5)*exp(1j*2*pi*rand);
rxPulse2 =  0.1*frequencyOffset(fmcwWave,fs,-fs/5)*exp(1j*2*pi*rand);
rxPulse2 = zeros(numel(rxPulse1),1);
rx= [zeros(1,1000)';rxPulse1] + [rxPulse2;zeros(1,1000)']; rx = rx +randn(numel(rx),1)*0.001;

rxmf = filter(conj(fmcwWave)/numel(fmcwWave),1,rx,[],1);
figure;plot(db(abs(rxmf)));


wc = (-10:10) * 2 * pi * fs/20;
t = 1/fs*(1:numel(rxmf));
carriers = exp(1j*wc.*t');

pulseDownConv = rxmf.*carriers;
% pulseFiltered = lpfIIR(pulseDownConv);
% filterBankMf = filter(conj(fmcwWave),1,pulseFiltered,[],1);
pulseFiltered = lpfIIR(pulseDownConv);
filterBankMf = filter(conj(fmcwWave)/numel(fmcwWave),1,pulseFiltered,[],1);

figure;mesh(db(abs(filterBankMf)))
figure;plot(mean(abs(filterBankMf),1))

