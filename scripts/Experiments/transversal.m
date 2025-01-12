clc;clear;close all;

c = 3e8;   % Propagation speed


max_range = 2000;       % Maximum unambiguous range
range_res = 50;         % Required range resolution
pulse_bw = 10*c/(2*range_res);    % Pulse bandwidth
pw = (2*range_res)/c;               % Pulse width
prf = c/(2*max_range);         % Pulse repetition frequency
fs = 1000*pulse_bw;              % Sampling rate
tx_peak_power = 8;           % Low powered

waveForm = phased.FMCWWaveform('SampleRate',fs,'SweepDirection','Triangle','SweepBandwidth',pulse_bw,'SweepTime',pw);
mfIdeal = waveForm.step';
mfIdeal = frequencyOffset(mfIdeal,fs,-pulse_bw/2);
mfIdeal = mfIdeal./sum(abs(mfIdeal));
mfbase = dsp.FIRFilter(mfIdeal);
mf1 = dsp.FIRFilter(fir_resample(mfIdeal,10));
mf2 = dsp.FIRFilter(fir_resample(mfIdeal,25));
mf3 = dsp.FIRFilter(fir_resample(mfIdeal,100));
subplot(2,2,1); plot(real(mfIdeal)); hold; plot(imag(mfIdeal));hold
title("Ideal Matched Filter");
subplot(2,2,2); stem(real(mf1.coeffs.Numerator(1:10:end))); hold; stem(imag(mf1.coeffs.Numerator(1:10:end)));hold
title("100 Tap transversal");
subplot(2,2,3); stem(real(mf2.coeffs.Numerator(1:25:end))); hold; stem(imag(mf2.coeffs.Numerator(1:25:end)));hold
title("40 Tap transversal");
subplot(2,2,4); stem(real(mf3.coeffs.Numerator(1:100:end))); hold; stem(imag(mf3.coeffs.Numerator(1:100:end)));hold
title("10 Tap transversal");

lpf = dsp.LowpassFilter("DesignForMinimumOrder",false,"FilterOrder",3,"SampleRate",fs,"PassbandFrequency",4*pulse_bw,"StopbandFrequency",2.2*pulse_bw,"FilterType","IIR");

fvtool(mfIdeal,mf1,mf2,mf3);

lpfmf1 = cascade(lpf,mf1);
lpfmf2 = cascade(lpf,mf2);
lpfmf3 = cascade(lpf,mf3);

% fvtool(lpf);

fvtool(mfbase,lpfmf1,lpfmf2,lpfmf3);

pulseInterval = zeros(1,50000);
pulseLocation = randi([1,24000]);
dopplerFreq = rand*1000-500;
pulseInterval(pulseLocation:pulseLocation+numel(mfIdeal)-1) = mfIdeal;
signalPower = db(max(abs(pulseInterval).^2),'power');
for i = 1:6
    pulseInterval_channel = exp(1i*(1:numel(pulseInterval))./(2*pi*fs)+rand*2*pi - pi).*pulseInterval;
    noisy_rx(:,i) = awgn(pulseInterval_channel,3,signalPower);    
end

figure;
plot(abs(pulseInterval_channel));
figure;
plot(abs(noisy_rx(:,end/2)));

ideal_output = mfbase(noisy_rx);
figure; plot(abs(ideal_output));
integrated_ideal = mean(abs(ideal_output),2);
figure; plot(integrated_ideal);
hold;
mf1out = lpfmf1(noisy_rx);
mf1Integrated = mean(abs(mf1out),2);
plot(mf1Integrated);

mf2out = lpfmf2(noisy_rx);
mf2Integrated = mean(abs(mf2out),2);
plot(mf2Integrated);

mf3out = lpfmf3(noisy_rx);
mf3Integrated = mean(abs(mf3out),2);
plot(mf3Integrated);

grid on
legend("Ideal Filter","100 Tap Interpolation","25 Tap Interpolation","10 Tap Interpolation")



function coeff_out = fir_resample(coeff_in,rate)
    ds_c = downsample(coeff_in,rate);
    us_c = upsample(ds_c,rate);
    % coeff_out = reshape(us_c * rate,size(coeff_in));
     coeff_out = us_c * rate;
end