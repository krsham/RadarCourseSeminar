## Introduction
- This seminar outline
- Introduce filter bank technologies by era
- Introduce variations of it and their pros and cons.
## What is filtering?
- Filtering is a signal processing technique used to modify or suppress unwanted parts of a signal, such as noise, or to extract useful parts of the signal, such as specific frequency components

## What are filtering applications?

- (1) One type of filter is a frequency-selective circuit that separates the radar signal from its background based on the signal frequency spectrum. Filters of this type are referred to as frequency(-selective) filters, classified as low-pass, high pass, bandpass, or band-stop filters. Depending on the center frequency, tuned filters are divided into radio-frequency (RF, or microwave) filters, intermediate-frequency (IF) filters, or video filters, and, depending on bandwidth, into narrowband or wideband filters. These are used primarily in the radar receiver. 
- (2) A second type of filter is a two-port device providing a required output based on its input signal. In this sense it is a major component of the radar signal processor. An optimum filter (which is termed a matched filter for a background of white noise) gives the highest signal-to-noise ratio at its out put, for given input signal energy. Signal processing filters are divided into analog, discrete, and digital filters (Fig. F21), based on the representation of input and output signals, and into linear and nonlinear filters based on the equations linking the input and output signals.
- (3) This type of filter is an algorithm processing a set of individual estimates (e.g., radar measurements obtained over successive time periods) with the objective of obtaining a new estimate for time tn. If tn is equal to or less than the cur rent time tj when the last measurement is made, the operation is termed smoothing (or interpolation); if tn > tj, it is termed prediction or extrapolation. The two basic types of filter per forming these operations are the α-β(-γ) filter and the Kalman filter.
## Evolution of filter technologies