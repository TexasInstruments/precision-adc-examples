%% Generate LPF Coefficients for PGA970
% This script generates 1st-order Butterworth Low Pass Filter coefficients
% for the PGA970 demodulator. It sweeps across cutoff frequencies and
% downsample rates to create a lookup table of filter configurations.
%
% Output: lpf.txt — Tab-delimited table with filter coefficients and
%                    register values (b1, a2) for hardware implementation.

clear all
close all

%% Configuration
% Modify these parameters to generate different coefficient tables

fSample = 1/(256e-6);           % Sample rate in Hz (3906.25 Hz)
minDS = 0.5;                    % Minimum downsample rate
maxDS = 1;                      % Maximum downsample rate
cfStart = 20;                   % Starting cutoff frequency (Hz)
cfEnd = 1000;                   % Maximum cutoff frequency (Hz)
cfStep = 10;                    % Cutoff frequency increment (Hz)

%% Generate LPF Coefficient Table
% Iterate through all combinations of cutoff frequencies and downsample rates

LPFs = [];
filterNum = 1;

cf = cfStart;
while (cf <= cfEnd)
    ds = minDS;  % Downsample rate
    while (ds <= maxDS)
        % Calculate output data rate in microseconds
        outputRate = ds / fSample * 1e6;

        % Calculate normalized cutoff frequency (in units of π rad/sample)
        % Normalized frequency = cutoff / (Nyquist frequency)
        % where Nyquist = fSample / (2 * ds)
        lEdge = cf / (fSample / ds / 2);

        % Design 1st-order Butterworth filter
        [b, a] = butter(1, lEdge);

        % Convert coefficients to 16-bit signed integer format for hardware
        % b1 = numerator coefficient (scaled by 2^15)
        % a2 = denominator coefficient (negated and scaled by 2^15)
        b1 = round(b(1) * 2^15);
        a2 = -round(a(2) * 2^15);

        % Store row: FilterNum | CutoffFreq | DS | OutputRate | b(1) | b(2) | a(1) | a(2) | b1 | a2 | cf/1000
        LPFs = [LPFs; [filterNum cf ds outputRate b a cf ds b1 a2 cf/1000]];

        filterNum = filterNum + 1;
        ds = ds + 0.5;
    end
    cf = cf + cfStep;
end

%% Save Coefficient Table
save lpf.txt LPFs -ascii -tabs -double

%% Visualization for Selected Filter
% Select a specific row from the LPF table to visualize its frequency response
% and step response

selectedRow = 197;  % MODIFY THIS to select a different row
b_coeff = [LPFs(selectedRow, 5), LPFs(selectedRow, 6)];
a_coeff = [LPFs(selectedRow, 7), LPFs(selectedRow, 8)];

% Calculate effective sampling rate for visualization
effectiveSampleRate = fSample / LPFs(selectedRow, 3);

% Plot frequency response (doubled cascade for 2nd-order response)
figure('Name', 'LPF Frequency Response', 'NumberTitle', 'off');
freqz(conv(b_coeff, b_coeff), conv(a_coeff, a_coeff), [], effectiveSampleRate);

% Plot step response (doubled cascade for 2nd-order response)
figure('Name', 'LPF Step Response', 'NumberTitle', 'off');
stepz(conv(b_coeff, b_coeff), conv(a_coeff, a_coeff), [], effectiveSampleRate);

%% Convert Selected Coefficients to Hexadecimal
% These hex values can be directly programmed into PGA970 registers
% DEMODx_LPF_B1 and DEMODx_LPF_A2

b1Hex = dec2hex(round(LPFs(selectedRow, 5) * 2^15));
a2Hex = dec2hex(-round(LPFs(selectedRow, 8) * 2^15));

fprintf('\nSelected Filter (Row %d):\n', selectedRow);
fprintf('  Cutoff Frequency: %.1f Hz\n', LPFs(selectedRow, 2));
fprintf('  b1 (decimal): %d\n', round(LPFs(selectedRow, 5) * 2^15));
fprintf('  a2 (decimal): %d\n', -round(LPFs(selectedRow, 8) * 2^15));
fprintf('  b1 (hex): 0x%s\n', b1Hex);
fprintf('  a2 (hex): 0x%s\n', a2Hex);
