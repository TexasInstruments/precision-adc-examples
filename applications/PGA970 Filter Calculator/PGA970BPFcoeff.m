%% Generate BPF Coefficients for PGA970
% This script generates 2nd-order Butterworth Bandpass Filter coefficients
% for the PGA970 demodulator. It sweeps across center frequencies and
% bandwidth values to create a lookup table of filter configurations.
%
% Output: bpf.txt — Tab-delimited table with filter coefficients and
%                    register values (b1, a2, a3) for hardware implementation.

clear all
close all

%% Configuration
% Modify these parameters to generate different coefficient tables

fSample = 1000e3;               % Sample rate in Hz (1 MHz)

% Define center frequencies to evaluate (Hz)
% Uncomment alternative to sweep multiple frequencies
centerFrequencies = 12500;
% centerFrequencies = [1500, 2500, 3500, 4500, 5500, 6500, 7500, 8500, ...
%                      9500, 10500, 11500, 12500, 13500, 14500, 15500, ...
%                      16500, 17500, 18500, 19500];

% Define bandwidths to evaluate (Hz)
bandwidths = [20, 100, 250];

%% Generate BPF Coefficient Table
% Iterate through all combinations of center frequencies and bandwidths

BPFs = [];
filterNum = 1;

for cf = centerFrequencies
    for bw = bandwidths
        % Calculate normalized band edges (normalized to π rad/sample)
        % Lower edge = (center_freq * 2 - bandwidth/2) / fSample
        % Upper edge = (center_freq * 2 + bandwidth/2) / fSample
        % Note: Multiply by 2 because Butterworth expects normalized freq in [0, 1]
        lEdge = (cf * 2 - bw) / fSample;
        uEdge = (cf * 2 + bw) / fSample;

        % Design 2nd-order Butterworth bandpass filter
        [b, a] = butter(1, [lEdge, uEdge]);

        % Convert coefficients to fixed-point integer format for hardware
        % b1 = numerator coefficient (scaled by 2^24)
        % a2 = denominator coefficient (negated and scaled by 2^23)
        % a3 = denominator coefficient (scaled by 2^24)
        b1 = round(b(1) * 2^24);
        a2 = -round(a(2) * 2^23);
        a3 = round(a(3) * 2^24);

        % Store row: FilterNum | CenterFreq | Bandwidth | b(1) | b(2) | b(3) | a(1) | a(2) | a(3) | cf | bw | b1 | a2 | a3 | cf/1000 | bw/1000
        BPFdata = [filterNum cf bw b a cf bw b1 a2 a3 cf/1000 bw/1000];
        BPFs = [BPFs; BPFdata];

        filterNum = filterNum + 1;
    end
end

%% Save Coefficient Table
save bpf.txt BPFs -ascii -tabs -double

%% Convert Selected Filter to Hexadecimal
% Select a specific row from the BPF table and convert its coefficients to hex
% These hex values can be directly programmed into PGA970 registers:
% DEMODx_BPF_B1, DEMODx_BPF_A2, and DEMODx_BPF_A3

selectedRow = 3;  % MODIFY THIS to select a different row

b1Hex = dec2hex(round(BPFs(selectedRow, 4) * 2^24));
a2Hex = dec2hex(-round(BPFs(selectedRow, 8) * 2^23));
a3Hex = dec2hex(round(BPFs(selectedRow, 9) * 2^24));

fprintf('\nSelected Filter (Row %d):\n', selectedRow);
fprintf('  Center Frequency: %.1f Hz\n', BPFs(selectedRow, 2));
fprintf('  Bandwidth: %.1f Hz\n', BPFs(selectedRow, 3));
fprintf('  b1 (decimal): %d\n', round(BPFs(selectedRow, 4) * 2^24));
fprintf('  a2 (decimal): %d\n', -round(BPFs(selectedRow, 8) * 2^23));
fprintf('  a3 (decimal): %d\n', round(BPFs(selectedRow, 9) * 2^24));
fprintf('  b1 (hex): 0x%s\n', b1Hex);
fprintf('  a2 (hex): 0x%s\n', a2Hex);
fprintf('  a3 (hex): 0x%s\n', a3Hex);



