# TODO

- [x] 1950s
- [x] 1960s-1980s
- [x] 1990s-2000s
- [x] 2000s-present

## 1950
To demonstrate the initial attempts at using filters to improve detection in radar systems, I have prepared few examples described below:
- Delay Line Canceller
- Chebyshev Canceller
- Staggered PRF
- Transversal Filtering

### Delay line canceller

For this example, the script [scripts/Experiments/delaylineFilterBank.m] was developed. This script is consisted of several stages. At the **generate_filters** stage, the delay line filter shall be generated using the user specified settings. Then, at the **plot_filter_resp** stage, the generated filter frequency response is plotted using the coefficients acquired at the previous stage. 
Then a scenario consisted of a plane which rotates around itself in a circular fashion is generated using the user specified settings. Moving in this manner, the target shall go through different relative speeds towards and away from the tracking radar. Thus allowing different doppler ranges to demonstrate the blind-speed effects on detection.
After performing the kinematic simulation of  the target the location and speed data generated from previous step is used for signal transmission and reception simulation. The radar shall generate a predefined rectangular pulse towards the target and receive the pulse, in addition using a predefined receiver, this operation allows addition of white noise to the received siganl.
After this step, a demo integration and pulse shaping is performed to show how received signal shall behave in ideal scenarios. 
Then, the previously unprocessed signals are added to a series of environmental clutter waves generated using a constant gamma model.
After this step the cluttered signal is again processed by receiver and shown to demonstrate how clutter signal can completely blind the radar. After that, a delay-line cancellers are applied to show the clutter cancellation effect and blind speed problem at different orders of delay line canceller.
To improve this a chebyshev polynomial is used to develop a high-pass filter in [scripts/Experiments/recursiveFilter.m]. Using the exact scenario implemented in the previous experiment, one can see the improved stability of the received signal and the reduced amount of blind doppler frequencies. However the Signal processing of the delay line canceller filter becomes more complicated.
To demonstrate the staggered prf response the [scripts/Experiments/staggeredPRF.m] is developed using the Shrader ratios specified in Skolnik Radar Systems.
### Matched Filter
After these steps, an experiment is performed in [scripts/Experiments/transversal.m] is used to show how initial matched filters were employed.
First a FMC waveform is generated using sample rate $f_s \gg BW_{fmcw}$ to allow fully capturing the frequency artifacts of the generated waveform. Then,4 downsampled versions of the waveform are generated to demonstrate the frequency domain. After that, LPF versions of generated signals is generated to show how downsampled matched filter(here employed as an interpolated FIR) can be effective just like the full response matched filter. For this step the initial signal is passed through every one of the filters an then, through a low pass filter. Then each filter ouput is plotted to show their equality.
