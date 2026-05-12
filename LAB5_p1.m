%(1)
%% Network Configuration for Controlling Two USRPs
% Computer side:
%   IP address: 192.168.10.1
%   Subnet mask: 255.255.255.0
% 
% USRP-2920 (TX):
%   Platform: N200/N210/USRP2
%   IP address: 192.168.10.2
% 
% USRP-2901 (RX):
%   Platform: B210
%   Serial Number: 34D9DC3
% 
% Explanation:
%   The computer's Ethernet interface is configured to the same subnet (192.168.10.x) as the USRP-2920,
%   so they can communicate directly over the wired Ethernet connection.
%   The USRP-2901 is connected via USB 3.0 and is identified by its serial number, not by IP.
%   MATLAB's UHD driver allows controlling both devices simultaneously.

%(2)
%% Strategy to Capture a Complete Frame
% To capture a complete frame:
% 1. The TX frame is repeated 10 times to make it much longer than one frame.
%    This ensures at least one full frame falls entirely inside the RX buffer.
% 2. At the receiver, we compute the matched filter output between the received signal and the known Short Training Sequence (STS). 
%    The STS has good autocorrelation properties.
% 3. When the matched filter peak exceeds a preset threshold, a frame is detected.
% 4. The STS start is estimated from the peak position.
% 5. The frame boundaries are determined by subtracting/adding the known padding, STS, LTS, and OFDM data lengths.
% 6. We check that both start and end indices lie within the received buffer to guarantee a complete capture.
% 
% This approach is robust against random arrival time of the frame inside the long repeated transmission.

%(3)
% parameters
FFT_size = 64;
cp_size = 16;
fs = 1e6;
sc_space = fs/FFT_size;
% 802.11a/g subcarrier allocation
active_sc  = [-26:-1 1:26];           % 52 active subcarriers
pilot_sc = [-21 -7 7 21];           % 4 pilot tones
data_sc  = setdiff(active_sc, pilot_sc); % 48 data tones
sc2idx = @(k) k + FFT_size/2 + 1;
data_idx = sc2idx(data_sc);

% construct the frame
sts = gen_sts();
lts = gen_lts();
num_ofdm_symbols = 100;
[ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, 4);
sts = sts / rms(sts);
lts = lts / rms(lts);
ofdm_data = ofdm_data / rms(ofdm_data);

pad_len = 500;
tx_frame = [zeros(pad_len,1); sts; lts; ofdm_data; zeros(pad_len,1)];
tx_frame = tx_frame/abs(max(tx_frame));
% transmit each frame with 10 copies
N_repeat = 10;
data = repmat(tx_frame, N_repeat, 1);

% set USRP
fc = 885e6;
tx_gain = 5;
rx_gain = 5;
rx_length = (N_repeat + 2)*length(tx_frame);
OFDM_sr = 1e6;
[radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr);

% USRP connection check
info = findsdru();
disp(info(1))
disp(info(2))
plot_td_signal(tx_frame, fs, 'Tx Wi-Fi OFDM Frame', 'Real');

% multi-tries transmission
% Matched filter for STS detection
match_filter = conj(flipud(sts));

% Variables for repeated transmission
buffer = zeros(rx_length, 1);
tunderrun = 0;
toverflow = 0;
start_point = 0;
success = 0;
retry_count = 0;

% parameter can be altered
maxattempts = 50;
threshold = 0.003 * length(sts) * mean(abs(sts).^2);
maxretries = 10;

while ~success && retry_count < maxretries
    retry_count = retry_count + 1;

    for attempt = 1:maxattempts
        % Transmit the generated frame
        tunderrun = radio_Tx(data);
    
        % Receive signal
        [received_signal, ~, toverflow] = step(radio_Rx);
        
        if toverflow
            continue;
        end
    
        % STS matched filter
        corr = abs(conv(received_signal, match_filter));
        maxval = max(corr);
    
        if maxval >= threshold
    
            % Store received signal
            buffer(:,1) = received_signal;
    
            % Find the approximate STS position
            [~, peak_idx] = max(corr);
    
            % Since the matched filter peak appears near the matched segment,
            % estimate the STS start point from the peak position
            sts_start = peak_idx - 160 + 1;
    
            % The transmitted frame has pad_len zeros before STS
            frame_start = sts_start - pad_len;
            frame_end = frame_start + length(tx_frame) - 1;
    
            start_point = frame_start;
            
            if frame_start < 1 || frame_end > length(received_signal)
                continue;
            end
            
            success = 1;
            break
        else
            % fprintf("Attempt %d: max corr = %.6f\n", attempt, maxval);
        end
    end
    
    if success
        fprintf("Good\n")
    else
        fprintf("Attempt TOO MANY TIMES, restarting usrp!\n");
        release(radio_Tx);
        release(radio_Rx);
        [radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr);
        continue;
    end
    
    % Extract received frame
    rx_frame = buffer(start_point:frame_end);
    
    % Extract OFDM symbols
    data_start = pad_len  + length(sts) + length(lts) + 1;
    rx_symbols = extractOFDMSymbols(rx_frame, data_start, FFT_size, cp_size, num_ofdm_symbols);
    
    % Demodulate data tones into bits
    rx_bits = zeros(2*length(data_sc), num_ofdm_symbols);
    lts_f_known = fftshift(fft(lts(33:96)));
    first_lts_start = pad_len + length(sts) + 33;
    rx_lts_1 = extractLTS(rx_frame, first_lts_start, 1, FFT_size);
    H1 = estimateChannelFromLTS(rx_lts_1, lts_f_known);
    
    rx_data = zeros(length(data_sc), num_ofdm_symbols);
    for k = 1:num_ofdm_symbols
        % Remove CP + FFT + fftshift
        Y = ofdmDemodSymbol(rx_symbols(:, k), FFT_size, cp_size);
        
        % Equalization
        X_hat = equalizeSymbol(Y, H1);
    
        % Only take data subcarriers, not pilots
        rx_data_syms_bn = X_hat(data_idx);
        rx_data_syms = rx_data_syms_bn / sqrt(mean(abs(rx_data_syms_bn).^2)); %%% normalized the power to 1
        rx_data(:, k) = rx_data_syms(:); 

        % 4-QAM demodulation
        rx_bits_k = qamdemod(rx_data_syms, 4, 'OutputType', 'bit','UnitAveragePower', true);
    
        rx_bits(:, k) = rx_bits_k(:);
    end
    
    % Check BER of this frame
    bit_errors = sum(rx_bits(:) ~= tx_bits(:));
    
    %%% discarding failed frame %%%
    ber = bit_errors / numel(tx_bits);
    fprintf("BER = %3f\n", ber);

    % if ber > 0.2
    %    fprintf("BER too high, NAK\n");
    %    success = 0;
    %    continue;
    % end 
end
frame_len = length(tx_frame);
region_names = {'STS', 'LTS', 'OFDM Data'};
region_ranges = [
    pad_len+1, pad_len+length(sts);                      % STS
    pad_len+length(sts)+1, pad_len+length(sts)+length(lts); % LTS
    pad_len+length(sts)+length(lts)+1, pad_len+length(sts)+length(lts)+length(ofdm_data) % Data
];
plot_td_signal(rx_frame, fs, 'Received Wi-Fi OFDM Frame', 'Real', region_names, region_ranges);

%(4)
plotConstellation(rx_data, 'received constellation without CFO');

%(5)
fs_CFO = OFDM_sr;

% Use STS repeated structure for CFO estimation
% STS has repetition period of 16 samples
D_sts = 16;

sts_start_in_frame = pad_len + 1;
rx_sts = rx_frame(sts_start_in_frame : sts_start_in_frame + length(sts) - 1);

P_sts = sum(conj(rx_sts(1:end-D_sts)) .* rx_sts(1+D_sts:end));
cfo_est = angle(P_sts) * fs_CFO / (2*pi*D_sts);

fprintf("Estimated CFO from STS = %.2f Hz\n", cfo_est);
%%
% CFO Estimation Process:
%   We use the STS for CFO estimation because:
%   - STS has a periodicity of D = 16 samples
%   - The phase rotation between two identical STS segments is proportional to the CFO
%   - CFO = angle(P) * fs / (2*pi*D) where P = sum( conj(rx_sts(1:end-D)) .* rx_sts(1+D:end) )
% 
%   LTS could also be used, but STS gives a wider estimation range because of its shorter repetition period (16 vs 64).
%   Here we only use STS for simplicity.

% % Description TBD

%(6)
% CFO correction
n = (0:length(rx_frame)-1).';
rx_frame_cfo = rx_frame .* exp(-1j*2*pi*cfo_est*n/fs_CFO);

% LTS positions inside rx_frame
first_lts_start = pad_len + length(sts) + 33;   % skip 32-sample CP of LTS

% Extract two received LTSs after CFO correction
rx_lts_1_cfo = extractLTS(rx_frame_cfo, first_lts_start, 1, FFT_size);
rx_lts_2_cfo = extractLTS(rx_frame_cfo, first_lts_start, 2, FFT_size);

% Estimate channel from two LTSs
H1_cfo = estimateChannelFromLTS(rx_lts_1_cfo, lts_f_known);
H2_cfo = estimateChannelFromLTS(rx_lts_2_cfo, lts_f_known);

% Average two LTS channel estimates
H_cfo = (H1_cfo + H2_cfo) / 2;

% Subcarrier axis after fftshift: -32, ..., 0, ..., 31
subcarrier_axis = (-FFT_size/2):(FFT_size/2-1);

% Only valid LTS subcarriers are nonzero in lts_f_known
valid_lts_idx = abs(lts_f_known) > 1e-12;

% For plotting, set invalid/null subcarriers to NaN
H_mag_plot = abs(H_cfo);
H_phase_plot = angle(H_cfo);

H_mag_plot(~valid_lts_idx) = NaN;
H_phase_plot(~valid_lts_idx) = NaN;

% Plot magnitude and phase of estimated channel
figure;

subplot(2,1,1);
plot(subcarrier_axis, H_mag_plot, '-o', 'LineWidth', 1.2);
grid on;
xlabel('Subcarrier Index');
ylabel('|H[k]|');
title('Estimated Channel Magnitude after CFO Correction');

subplot(2,1,2);
plot(subcarrier_axis, unwrap(H_phase_plot), '-o', 'LineWidth', 1.2);
grid on;
xlabel('Subcarrier Index');
ylabel('Phase of H[k] (rad)');
title('Estimated Channel Phase after CFO Correction');

% % Description TBD

%(7)
rx_symbols_cfo = extractOFDMSymbols(rx_frame_cfo, data_start, FFT_size, cp_size, num_ofdm_symbols);
rx_data_cfo = zeros(length(data_sc), num_ofdm_symbols);

for k = 1:num_ofdm_symbols
    % Remove CP + FFT + fftshift
    Y = ofdmDemodSymbol(rx_symbols_cfo(:, k), FFT_size, cp_size);
    
    % Equalization
    X_cfo = equalizeSymbol(Y, H_cfo);
    
    % Only take data subcarriers, not pilots
    rx_data_syms_bn_cfo = X_cfo(data_idx);
    rx_data_syms_cfo = rx_data_syms_bn_cfo / sqrt(mean(abs(rx_data_syms_bn_cfo).^2)); %%% normalized the power to 1
    rx_data_cfo(:, k) = rx_data_syms_cfo(:);
end
plotConstellation(rx_data_cfo, 'received constellation after CFO');

% % Description TBD

%(8)
pilot_idx = sc2idx(pilot_sc);

rx_data_pilot = zeros(length(data_sc), num_ofdm_symbols);
tracked_phase = zeros(1, num_ofdm_symbols);

for k = 1:num_ofdm_symbols
    % Remove CP + FFT + fftshift from CFO-corrected frame
    Y = ofdmDemodSymbol(rx_symbols_cfo(:, k), FFT_size, cp_size);

    % Equalization using channel estimated from LTS
    X_hat = equalizeSymbol(Y, H_cfo);

    % Received pilot tones after equalization
    rx_pilot = X_hat(pilot_idx);

    % Known transmitted pilot tones
    tx_pilot = pilot_syms(:, k);

    % Estimate common phase error using four pilots
    theta = angle(sum(rx_pilot .* conj(tx_pilot)));

    tracked_phase(k) = theta;

    % Correct all subcarriers in this OFDM symbol
    X_hat_pilot = X_hat * exp(-1j * theta);
    rx_data_pilot_bn = X_hat_pilot(data_idx);
    rx_data_pilot_k = rx_data_pilot_bn / sqrt(mean(abs(rx_data_pilot_bn).^2));
    % Store only data tones
    rx_data_pilot(:, k) = rx_data_pilot_k(:);
end

% Normalize once for plotting
plotConstellation(rx_data_pilot, 'pilot-assisted received constellation', tx_data_syms(:));

% % Description TBD

%(9)
rx_bits = zeros(2*length(data_sc), num_ofdm_symbols);

for k = 1:num_ofdm_symbols
    rx_data_pilot_k = rx_data_pilot(:, k);
    rx_bits_k = qamdemod(rx_data_pilot_k, 4, 'OutputType', 'bit','UnitAveragePower', true);        
    rx_bits(:, k) = rx_bits_k(:);
end

% calculate BER
bit_errors = sum(rx_bits(:) ~= tx_bits(:));
ber = bit_errors / numel(tx_bits);
fprintf("BER = %3f\n", ber);

%(10)
%(10)
% =========================================================================
% (10) Change the modulation to 16-QAM and repeat the experiment
% =========================================================================
% 這題要把 modulation 改成 16-QAM，重新傳送與接收一個 frame。
% 然後重複：
%   1. frame synchronization
%   2. CFO estimation
%   3. CFO correction
%   4. channel estimation
%   5. equalization
%   6. pilot-assisted residual phase correction
%   7. plot constellation
%
% 注意：
%   這裡沿用前面 Q3~Q9 的架構，只是 qam_num 改成 16。

qam_num_16 = 16;

% -------------------------------------------------------------------------
% Generate 16-QAM OFDM frame
% -------------------------------------------------------------------------
[ofdm_data_16, tx_bits_16, tx_data_syms_16, pilot_syms_16] = ...
    gen_ofdm_data(num_ofdm_symbols, qam_num_16);

% Normalize power of STS, LTS, and OFDM data
sts_16 = sts / rms(sts);
lts_16 = lts / rms(lts);
ofdm_data_16 = ofdm_data_16 / rms(ofdm_data_16);

% Construct TX frame
tx_frame_16 = [
    zeros(pad_len, 1);
    sts_16;
    lts_16;
    ofdm_data_16;
    zeros(pad_len, 1)
];

% Normalize TX frame peak power
tx_frame_16 = tx_frame_16 / max(abs(tx_frame_16));

% Repeat frame for easier complete capture
data_16 = repmat(tx_frame_16, N_repeat, 1);

% RX capture length
rx_length_16 = (N_repeat + 2) * length(tx_frame_16);

fprintf("\n================ Q10: 16-QAM Transmission ================\n");
fprintf("16-QAM frame length = %d samples\n", length(tx_frame_16));
fprintf("16-QAM rx length    = %d samples\n", rx_length_16);

% -------------------------------------------------------------------------
% Re-initialize USRP for 16-QAM experiment
% -------------------------------------------------------------------------
[radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length_16, OFDM_sr);

% -------------------------------------------------------------------------
% Multi-tries transmission and frame detection
% -------------------------------------------------------------------------
match_filter_16 = conj(flipud(sts_16));

buffer_16 = zeros(rx_length_16, 1);
success_16 = 0;
retry_count_16 = 0;

maxattempts_16 = 50;
maxretries_16 = 10;

% Threshold can be tuned if frame detection fails
threshold_16 = 0.003 * length(sts_16) * mean(abs(sts_16).^2);

while ~success_16 && retry_count_16 < maxretries_16

    retry_count_16 = retry_count_16 + 1;

    for attempt = 1:maxattempts_16

        fprintf("16-QAM attempt %d / %d, retry %d / %d\n", ...
            attempt, maxattempts_16, retry_count_16, maxretries_16);

        % Transmit repeated 16-QAM frame
        tunderrun = radio_Tx(data_16);

        if tunderrun
            fprintf("TX underrun happens.\n");
        end

        % Receive signal
        [received_signal_16, ~, toverflow] = step(radio_Rx);

        if toverflow
            fprintf("RX overflow happens. Try next attempt.\n");
            continue;
        end

        % STS matched filter for synchronization
        corr_16 = abs(conv(received_signal_16, match_filter_16));
        maxval_16 = max(corr_16);

        fprintf("max corr = %.6f, rx max = %.4f, rx rms = %.4f\n", ...
            maxval_16, max(abs(received_signal_16)), rms(received_signal_16));

        if maxval_16 >= threshold_16

            buffer_16(:, 1) = received_signal_16;

            % Find STS position
            [~, peak_idx_16] = max(corr_16);

            % STS length = 160 samples
            sts_start_16 = peak_idx_16 - length(sts_16) + 1;

            % TX frame has pad_len zeros before STS
            frame_start_16 = sts_start_16 - pad_len;
            frame_end_16 = frame_start_16 + length(tx_frame_16) - 1;

            % Check whether a complete frame is captured
            if frame_start_16 < 1 || frame_end_16 > length(received_signal_16)
                fprintf("Detected frame is incomplete. Try next attempt.\n");
                continue;
            end

            success_16 = 1;
            break;
        end
    end

    if success_16
        fprintf("16-QAM frame detected successfully.\n");
    else
        fprintf("Too many failed attempts. Restarting USRP...\n");

        release(radio_Tx);
        release(radio_Rx);

        [radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length_16, OFDM_sr);
    end
end

release(radio_Tx);
release(radio_Rx);

if ~success_16
    error("Q10 failed: Cannot detect complete 16-QAM frame.");
end

% -------------------------------------------------------------------------
% Extract the received 16-QAM frame
% -------------------------------------------------------------------------
rx_frame_16 = buffer_16(frame_start_16:frame_end_16);

fprintf("16-QAM frame_start = %d\n", frame_start_16);
fprintf("16-QAM frame_end   = %d\n", frame_end_16);

% Plot received 16-QAM time-domain frame
plot_td_signal(rx_frame_16, fs, ...
    'Q10: Received 16-QAM OFDM Frame', ...
    'Real');

% -------------------------------------------------------------------------
% CFO estimation using STS
% -------------------------------------------------------------------------
fs_CFO = OFDM_sr;
D_sts = 16;

sts_start_in_frame = pad_len + 1;

rx_sts_16 = rx_frame_16( ...
    sts_start_in_frame : sts_start_in_frame + length(sts_16) - 1);

P_sts_16 = sum(conj(rx_sts_16(1:end-D_sts)) .* rx_sts_16(1+D_sts:end));

cfo_est_16 = angle(P_sts_16) * fs_CFO / (2*pi*D_sts);

fprintf("Estimated CFO from STS for 16-QAM = %.2f Hz\n", cfo_est_16);

% -------------------------------------------------------------------------
% CFO correction
% -------------------------------------------------------------------------
n_16 = (0:length(rx_frame_16)-1).';

rx_frame_cfo_16 = rx_frame_16 .* exp(-1j * 2*pi * cfo_est_16 * n_16 / fs_CFO);

% -------------------------------------------------------------------------
% Channel estimation using LTS after CFO correction
% -------------------------------------------------------------------------
% Known LTS in frequency domain
lts_f_known_16 = fftshift(fft(lts_16(33:96)));

% First LTS body starts after:
%   pad_len zeros + STS + 32-sample LTS CP
first_lts_start_16 = pad_len + length(sts_16) + 33;

rx_lts_1_cfo_16 = extractLTS(rx_frame_cfo_16, first_lts_start_16, 1, FFT_size);
rx_lts_2_cfo_16 = extractLTS(rx_frame_cfo_16, first_lts_start_16, 2, FFT_size);

H1_cfo_16 = estimateChannelFromLTS(rx_lts_1_cfo_16, lts_f_known_16);
H2_cfo_16 = estimateChannelFromLTS(rx_lts_2_cfo_16, lts_f_known_16);

% Average two LTS channel estimates
H_cfo_16 = (H1_cfo_16 + H2_cfo_16) / 2;

% -------------------------------------------------------------------------
% Extract OFDM symbols after CFO correction
% -------------------------------------------------------------------------
data_start_16 = pad_len + length(sts_16) + length(lts_16) + 1;

rx_symbols_cfo_16 = extractOFDMSymbols( ...
    rx_frame_cfo_16, ...
    data_start_16, ...
    FFT_size, ...
    cp_size, ...
    num_ofdm_symbols);

% -------------------------------------------------------------------------
% Equalization without pilot-assisted correction
% -------------------------------------------------------------------------
rx_data_cfo_16 = zeros(length(data_sc), num_ofdm_symbols);

for k = 1:num_ofdm_symbols

    % Remove CP + FFT + fftshift
    Y_16 = ofdmDemodSymbol(rx_symbols_cfo_16(:, k), FFT_size, cp_size);

    % Equalization
    X_cfo_16 = equalizeSymbol(Y_16, H_cfo_16);

    % Take only data subcarriers
    rx_data_syms_bn_16 = X_cfo_16(data_idx);

    % Normalize constellation power for plotting and demodulation
    rx_data_syms_cfo_16 = rx_data_syms_bn_16 / ...
        sqrt(mean(abs(rx_data_syms_bn_16).^2));

    rx_data_cfo_16(:, k) = rx_data_syms_cfo_16(:);
end

plotConstellation(rx_data_cfo_16, ...
    'Q10: 16-QAM Constellation after CFO Correction and Equalization');

% -------------------------------------------------------------------------
% Pilot-assisted residual phase correction
% -------------------------------------------------------------------------
pilot_idx = sc2idx(pilot_sc);

rx_data_pilot_16 = zeros(length(data_sc), num_ofdm_symbols);
tracked_phase_16 = zeros(1, num_ofdm_symbols);

for k = 1:num_ofdm_symbols

    % Remove CP + FFT + fftshift from CFO-corrected frame
    Y_16 = ofdmDemodSymbol(rx_symbols_cfo_16(:, k), FFT_size, cp_size);

    % Equalization using channel estimated from LTS
    X_hat_16 = equalizeSymbol(Y_16, H_cfo_16);

    % Received pilot tones after equalization
    rx_pilot_16 = X_hat_16(pilot_idx);

    % Known transmitted pilot tones
    tx_pilot_16 = pilot_syms_16(:, k);

    % Estimate common phase error using four pilots
    theta_16 = angle(sum(rx_pilot_16 .* conj(tx_pilot_16)));

    tracked_phase_16(k) = theta_16;

    % Correct all subcarriers in this OFDM symbol
    X_hat_pilot_16 = X_hat_16 * exp(-1j * theta_16);

    % Take only data tones
    rx_data_pilot_bn_16 = X_hat_pilot_16(data_idx);

    % Normalize power
    rx_data_pilot_k_16 = rx_data_pilot_bn_16 / ...
        sqrt(mean(abs(rx_data_pilot_bn_16).^2));

    rx_data_pilot_16(:, k) = rx_data_pilot_k_16(:);
end

plotConstellation(rx_data_pilot_16, ...
    'Q10: 16-QAM Pilot-Assisted Received Constellation', ...
    tx_data_syms_16(:));

fprintf("Q10 finished: 16-QAM CFO correction, equalization, and pilot-assisted constellation plotted.\n");

%(11)
% =========================================================================
% (11) Calculate BER for 16-QAM and compare with 4-QAM
% =========================================================================
% Q11 要做的事情：
%   1. 將 Q10 pilot-assisted correction 後的 16-QAM symbols 解調成 bits
%   2. 跟原本傳送的 tx_bits_16 比較
%   3. 算出 16-QAM BER
%   4. 跟 Q9 的 4-QAM BER 比較

% -------------------------------------------------------------------------
% Save 4-QAM BER from Q9
% -------------------------------------------------------------------------
% 前面 Q9 算完後，變數 ber 代表 4-QAM BER。
% 為了避免後面被覆蓋，先存成 ber_4qam。
if exist('ber_4qam', 'var') == 0
    ber_4qam = ber;
end

% -------------------------------------------------------------------------
% Demodulate 16-QAM received symbols
% -------------------------------------------------------------------------
bits_per_symbol_16 = log2(qam_num_16);

% rx_data_pilot_16 size:
%   length(data_sc) x num_ofdm_symbols
%
% 對每個 OFDM symbol 分別做 16-QAM demodulation。
rx_bits_16 = zeros(bits_per_symbol_16 * length(data_sc), num_ofdm_symbols);

for k = 1:num_ofdm_symbols

    % 取出第 k 個 OFDM symbol 的 data tones
    rx_data_pilot_16_k = rx_data_pilot_16(:, k);

    % 16-QAM demodulation
    rx_bits_16_k = qamdemod(rx_data_pilot_16_k, qam_num_16, ...
        'OutputType', 'bit', ...
        'UnitAveragePower', true);

    % 存起來
    rx_bits_16(:, k) = rx_bits_16_k(:);
end

%-------------------------------------------------------------------------
% Calculate 16-QAM BER
% -------------------------------------------------------------------------
% tx_bits_16 是 Q10 產生的 transmitted bits
bit_errors_16 = sum(rx_bits_16(:) ~= tx_bits_16(:));

ber_16qam = bit_errors_16 / numel(tx_bits_16);

% -------------------------------------------------------------------------
% Print results
% -------------------------------------------------------------------------
fprintf("\n================ Q11 BER Comparison ================\n");
fprintf("4-QAM  BER from Q9  = %.6f\n", ber_4qam);
fprintf("16-QAM BER from Q11 = %.6f\n", ber_16qam);
fprintf("16-QAM bit errors   = %d / %d bits\n", bit_errors_16, numel(tx_bits_16));

if ber_16qam > ber_4qam
    fprintf("Observation: 16-QAM BER is higher than 4-QAM BER.\n");
    fprintf("Reason: 16-QAM constellation points are closer, so it is more sensitive to noise, residual CFO, and channel estimation error.\n");
elseif ber_16qam < ber_4qam
    fprintf("Observation: 16-QAM BER is lower than 4-QAM BER in this trial.\n");
    fprintf("This may happen due to channel variation, frame selection, or random transmission conditions.\n");
else
    fprintf("Observation: 16-QAM BER is the same as 4-QAM BER in this trial.\n");
end

% -------------------------------------------------------------------------
% Optional: bar plot comparing 4-QAM and 16-QAM BER
% -------------------------------------------------------------------------
figure;
bar([ber_4qam, ber_16qam]);
grid on;
set(gca, 'XTickLabel', {'4-QAM', '16-QAM'});
ylabel('BER');
title('Q11: BER Comparison between 4-QAM and 16-QAM');

% (12)
validtrans = 0;
frame_num = 20;
BER_plot = zeros(frame_num,1);
% parameters for Q(13)
M = 16;
bits_per_qam = log2(M);        
num_data_sc = length(data_sc); 
sc_err_cnt = zeros(num_data_sc, 1);
sc_bit_cnt = zeros(num_data_sc, 1);

while validtrans < frame_num
    [ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, 16);
    ofdm_data = ofdm_data / rms(ofdm_data);
    
    pad_len = 500;
    tx_frame = [zeros(pad_len,1); sts; lts; ofdm_data; zeros(pad_len,1)];
    tx_frame = tx_frame/abs(max(tx_frame));

    % transmit each frame with 10 copies
    N_repeat = 10;
    data = repmat(tx_frame, N_repeat, 1);
    % multi-tries transmission
    % Matched filter for STS detection
    match_filter = conj(flipud(sts));
    
    % Variables for repeated transmission
    buffer = zeros(rx_length, 1);
    tunderrun = 0;
    toverflow = 0;
    start_point = 0;
    success = 0;
    retry_count = 0;
    
    % parameter can be altered
    maxattempts = 50;
    threshold = 0.003 * length(sts) * mean(abs(sts).^2);
 
    for attempt = 1:maxattempts
        % Transmit the generated frame
        tunderrun = radio_Tx(data);
    
        % Receive signal
        [received_signal, ~, toverflow] = step(radio_Rx);
        
        if toverflow
            continue;
        end
    
        % STS matched filter
        corr = abs(conv(received_signal, match_filter));
        maxval = max(corr);
    
        if maxval >= threshold
    
            % Store received signal
            buffer(:,1) = received_signal;
    
            % Find the approximate STS position
            [~, peak_idx] = max(corr);
    
            % Since the matched filter peak appears near the matched segment,
            % estimate the STS start point from the peak position
            sts_start = peak_idx - 160 + 1;
    
            % The transmitted frame has pad_len zeros before STS
            frame_start = sts_start - pad_len;
            frame_end = frame_start + length(tx_frame) - 1;
    
            start_point = frame_start;
            
            if frame_start < 1 || frame_end > length(received_signal)
                continue;
            end
            
            success = 1;
            break
        else
            % fprintf("Attempt %d: max corr = %.6f\n", attempt, maxval);
        end
    end
    
    if success
        fprintf("GON GON\n")
    else
        fprintf("Attempt TOO MANY TIMES, restarting usrp!\n");
        release(radio_Tx);
        release(radio_Rx);
        [radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr);
        continue;
    end
    
    % Extract received frame
    rx_frame = buffer(start_point:frame_end);
    
    % Extract OFDM symbols
    data_start = pad_len  + length(sts) + length(lts) + 1;
    rx_symbols = extractOFDMSymbols(rx_frame, data_start, FFT_size, cp_size, num_ofdm_symbols);
    
    % Demodulate data tones into bits
    rx_bits = zeros(4*length(data_sc), num_ofdm_symbols);
    lts_f_known = fftshift(fft(lts(33:96)));
    first_lts_start = pad_len + length(sts) + 33;
    rx_lts_1 = extractLTS(rx_frame, first_lts_start, 1, FFT_size);
    H1 = estimateChannelFromLTS(rx_lts_1, lts_f_known);
    
    rx_data = zeros(length(data_sc), num_ofdm_symbols);
    for k = 1:num_ofdm_symbols
        % Remove CP + FFT + fftshift
        Y = ofdmDemodSymbol(rx_symbols(:, k), FFT_size, cp_size);
        
        % Equalization
        X_hat = equalizeSymbol(Y, H1);
    
        % Only take data subcarriers, not pilots
        rx_data_syms_bn = X_hat(data_idx);
        rx_data_syms = rx_data_syms_bn / sqrt(mean(abs(rx_data_syms_bn).^2)); %%% normalized the power to 1
        rx_data(:, k) = rx_data_syms(:); 

        % 4-QAM demodulation
        rx_bits_k = qamdemod(rx_data_syms, 16, 'OutputType', 'bit','UnitAveragePower', true);
    
        rx_bits(:, k) = rx_bits_k(:);
    end
    
    % Check BER of this frame
    bit_errors = sum(rx_bits(:) ~= tx_bits(:));
    
    %%% discarding ass frame %%%
    ber = bit_errors / numel(tx_bits);
    fprintf("BER = %3f\n", ber);
   
    frame_idx = validtrans + 1;

    BER_plot(frame_idx) = ber;
    
    % Calculate BER for each data subcarrier in this frame
    for sc_i = 1:num_data_sc
        bit_row_start = (sc_i - 1) * bits_per_qam + 1;
        bit_row_end   = sc_i * bits_per_qam;
    
        rx_sc_bits = rx_bits(bit_row_start:bit_row_end, :); % 4 x 100
        tx_sc_bits = tx_bits(bit_row_start:bit_row_end, :); % 4 x 100
    
        sc_err_cnt(sc_i) = sc_err_cnt(sc_i) + sum(rx_sc_bits ~= tx_sc_bits, 'all');
        sc_bit_cnt(sc_i) = sc_bit_cnt(sc_i) + numel(tx_sc_bits);
    end

    % if ber > 0.2
    %    fprintf("BER too high, NAK\n");
    %    success = 0;
    %    continue;
    % end 
    validtrans = validtrans + 1;
    % plot_td_signal(rx_frame, fs, 'Received Wi-Fi OFDM Frame', 'Real');
end

frame_axis = 1:validtrans;

figure;
plot(frame_axis, BER_plot(1:validtrans), '-o', 'LineWidth', 1.5);
grid on;
xlabel('Frame Number');
ylabel('BER');
title('BER per Frame');

total_BER = mean(BER_plot(1:validtrans));
fprintf("Total BER over %d valid frames = %.6f\n", validtrans, total_BER);

%(13)
BER_per_subcarrier = sc_err_cnt ./ sc_bit_cnt;  

figure;
stem(data_sc, BER_per_subcarrier, 'filled', 'LineWidth', 1.2);
grid on;
xlabel('Subcarrier Index');
ylabel('BER');
title('BER per Subcarrier over 20 Frames');

% % description TBD

%(14)
distance_num = 3;
BER_distance = zeros(distance_num,1);
distance_record = zeros(distance_num, 1);

for d_idx = 1:distance_num
    % initialization for every distance
    validtrans = 0;
    % reset usrp for each case
    release(radio_Tx);
    release(radio_Rx);
    [radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr);

    % TODO1: input the distance from the computer
    fprintf("\n====================================\n");
    fprintf("Distance test %d / %d\n", d_idx, distance_num);
    fprintf("Move TX/RX devices now.\n");
    fprintf("After measuring the distance, enter it below.\n");
    fprintf("====================================\n");

    distance_record(d_idx) = input("Enter measured distance in meters: ");
    fprintf("Start transmitting 20 frames at distance = %.2f m\n", distance_record(d_idx));

    while validtrans < frame_num
        [ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, 16);
        ofdm_data = ofdm_data / rms(ofdm_data);
        
        pad_len = 500;
        tx_frame = [zeros(pad_len,1); sts; lts; ofdm_data; zeros(pad_len,1)];
        tx_frame = tx_frame/abs(max(tx_frame));
    
        % transmit each frame with 10 copies
        N_repeat = 10;
        data = repmat(tx_frame, N_repeat, 1);
        % multi-tries transmission
        % Matched filter for STS detection
        match_filter = conj(flipud(sts));
        
        % Variables for repeated transmission
        buffer = zeros(rx_length, 1);
        tunderrun = 0;
        toverflow = 0;
        start_point = 0;
        success = 0;
        retry_count = 0;
        
        % parameter can be altered
        maxattempts = 50;
        threshold = 0.003 * length(sts) * mean(abs(sts).^2);
     
        for attempt = 1:maxattempts
            % Transmit the generated frame
            tunderrun = radio_Tx(data);
        
            % Receive signal
            [received_signal, ~, toverflow] = step(radio_Rx);
            
            if toverflow
                continue;
            end
        
            % STS matched filter
            corr = abs(conv(received_signal, match_filter));
            maxval = max(corr);
        
            if maxval >= threshold
        
                % Store received signal
                buffer(:,1) = received_signal;
        
                % Find the approximate STS position
                [~, peak_idx] = max(corr);
        
                % Since the matched filter peak appears near the matched segment,
                % estimate the STS start point from the peak position
                sts_start = peak_idx - 160 + 1;
        
                % The transmitted frame has pad_len zeros before STS
                frame_start = sts_start - pad_len;
                frame_end = frame_start + length(tx_frame) - 1;
        
                start_point = frame_start;
                
                if frame_start < 1 || frame_end > length(received_signal)
                    continue;
                end
                
                success = 1;
                break
            else
                % fprintf("Attempt %d: max corr = %.6f\n", attempt, maxval);
            end
        end
        
        if success
            fprintf("GON GON\n")
        else
            fprintf("Attempt TOO MANY TIMES, restarting usrp!\n");
            release(radio_Tx);
            release(radio_Rx);
            [radio_Tx, radio_Rx] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr);
            continue;
        end
        
        % Extract received frame
        rx_frame = buffer(start_point:frame_end);
        
        % Extract OFDM symbols
        data_start = pad_len  + length(sts) + length(lts) + 1;
        rx_symbols = extractOFDMSymbols(rx_frame, data_start, FFT_size, cp_size, num_ofdm_symbols);
        
        % Demodulate data tones into bits
        rx_bits = zeros(4*length(data_sc), num_ofdm_symbols);
        lts_f_known = fftshift(fft(lts(33:96)));
        first_lts_start = pad_len + length(sts) + 33;
        rx_lts_1 = extractLTS(rx_frame, first_lts_start, 1, FFT_size);
        H1 = estimateChannelFromLTS(rx_lts_1, lts_f_known);
        
        rx_data = zeros(length(data_sc), num_ofdm_symbols);
        for k = 1:num_ofdm_symbols
            % Remove CP + FFT + fftshift
            Y = ofdmDemodSymbol(rx_symbols(:, k), FFT_size, cp_size);
            
            % Equalization
            X_hat = equalizeSymbol(Y, H1);
        
            % Only take data subcarriers, not pilots
            rx_data_syms_bn = X_hat(data_idx);
            
            rx_data_syms = rx_data_syms_bn / sqrt(mean(abs(rx_data_syms_bn).^2)); %%% normalized the power to 1
            rx_data(:, k) = rx_data_syms(:); 
    
            % 4-QAM demodulation
            rx_bits_k = qamdemod(rx_data_syms, 16, 'OutputType', 'bit','UnitAveragePower', true);
        
            rx_bits(:, k) = rx_bits_k(:);
        end
        
        % Check BER of this frame
        bit_errors = sum(rx_bits(:) ~= tx_bits(:));
        
        %%% discarding ass frame %%%
        ber = bit_errors / numel(tx_bits);
        fprintf("BER = %3f\n", ber);
       
        frame_idx = validtrans + 1;
    
        BER_plot(frame_idx) = ber;
        
        % Calculate BER for each data subcarrier in this frame
        for sc_i = 1:num_data_sc
            bit_row_start = (sc_i - 1) * bits_per_qam + 1;
            bit_row_end   = sc_i * bits_per_qam;
        
            rx_sc_bits = rx_bits(bit_row_start:bit_row_end, :); % 4 x 100
            tx_sc_bits = tx_bits(bit_row_start:bit_row_end, :); % 4 x 100
        
            sc_err_cnt(sc_i) = sc_err_cnt(sc_i) + sum(rx_sc_bits ~= tx_sc_bits, 'all');
            sc_bit_cnt(sc_i) = sc_bit_cnt(sc_i) + numel(tx_sc_bits);
        end
    
        % if ber > 0.2
        %    fprintf("BER too high, NAK\n");
        %    success = 0;
        %    continue;
        % end 
        validtrans = validtrans + 1;
        % plot_td_signal(rx_frame, fs, 'Received Wi-Fi OFDM Frame', 'Real');
    end
    total_BER = mean(BER_plot(1:validtrans));
    % TODO2: store total BER at this distance
    BER_distance(d_idx) = total_BER;
    fprintf("Finished distance %.2f m, total BER = %.6f\n", distance_record(d_idx), BER_distance(d_idx));
end

% TODO3: plot BER vs distance
figure;
scatter(distance_record, BER_distance);
grid on;
xlabel('Transmission Distance (m)');
ylabel('Total BER');
title('Total BER vs Transmission Distance');


function [tx_usrp, rx_usrp] = USRP_init(fc, tx_gain, rx_gain, rx_length, OFDM_sr)
    inte_factor = 100e6 / OFDM_sr;
    deci_factor = 20e6 / OFDM_sr; 

    tx_usrp = comm.SDRuTransmitter( ...
        'Platform',            'N200/N210/USRP2', ...
        'IPAddress',           '192.168.10.2', ...
        'CenterFrequency',     fc, ...
        "MasterClockRate",     100e6, ...
        'InterpolationFactor',  inte_factor, ...
        'Gain',                tx_gain);

    rx_usrp = comm.SDRuReceiver( ...
        'Platform',            'B210', ...
        'SerialNum',           '34D9DC3', ...
        'CenterFrequency',     fc, ...
        'Gain',                rx_gain, ...
        'SamplesPerFrame',     rx_length, ...
        "MasterClockRate",     20e6, ...
        'DecimationFactor',    deci_factor, ...
        'OutputDataType',      'double');
end

% gen_sts
% Usage:
%   sts = gen_sts()
% Input:
%   None
% Output:
%   sts : time-domain 802.11a/g short training sequence, length = 160 samples
function sts = gen_sts()
    FFT_size = 64;

    short_sc = [-24 -20 -16 -12 -8 -4 4 8 12 16 20 24];
    sts_val = sqrt(13/6) * [1+1j, -1-1j,  1+1j, -1-1j, -1-1j,  1+1j, -1-1j, -1-1j, 1+1j,  1+1j,  1+1j,  1+1j];

    sts_f = zeros(FFT_size, 1);
    sts_f(short_sc + FFT_size/2 + 1) = sts_val;

    sts_64 = ifft(ifftshift(sts_f), FFT_size);
    sts_16 = sts_64(1:16);
    sts = repmat(sts_16, 10, 1);
end


% gen_lts
% Usage:
%   lts = gen_lts()
% Input:
%   None
% Output:
%   lts : time-domain 802.11a/g long training sequence, length = 160 samples
%         structure = [32-sample CP; 64-sample LTS; 64-sample LTS]
function lts = gen_lts()
    FFT_size = 64;

    lts_sc = -26:26;
    lts_val = [1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, ...
    -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  0,  1, -1, -1, ...
     1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1, ...
     1, -1,  1, -1,  1,  1,  1,  1];

    lts_f = zeros(FFT_size, 1);
    lts_f(lts_sc + FFT_size/2 + 1) = lts_val;

    lts_64 = ifft(ifftshift(lts_f), FFT_size);
    lts = [lts_64(end-31:end); lts_64; lts_64];
end


% gen_ofdm_symbol
% Usage:
%   [x_cp, data_bits, data_sym, pilot_sym] = gen_ofdm_symbol(qam_num)
% Input:
%   qam_num     :order of qam (4/16)
% Output:
%   x_cp      : one OFDM symbol with cyclic prefix, length = 80 samples
%   data_bits : transmitted bits on data subcarriers
%   data_sym  : transmitted 4-QAM symbols on data subcarriers
%   pilot_sym : pilot symbols on pilot subcarriers
function [x_cp, data_bits, data_sym, pilot_sym] = gen_ofdm_symbol(qam_num)
    FFT_size = 64;
    cp_size = 16;
    
    % 802.11a/g subcarrier allocation
    active_sc  = [-26:-1 1:26];           % 52 active subcarriers
    pilot_sc = [-21 -7 7 21];           % 4 pilot tones
    data_sc  = setdiff(active_sc, pilot_sc); % 48 data tones
    
    % 1. Generate pilot symbols (BPSK)
    pilot_bits = randi([0 1], length(pilot_sc), 1);
    pilot_sym = 2*pilot_bits - 1;

    % 2. Generate data symbols (4-QAM)
    bit = log2(qam_num);
    data_bits = randi([0 1], length(data_sc)*bit, 1);
    data_sym = qammod(data_bits, qam_num , 'InputType', 'bit', 'UnitAveragePower', true);

    % 3. Generate OFDM symbol
    Xc = zeros(FFT_size, 1);
    sc2idx = @(k) k + FFT_size/2 + 1;

    % map pilot & data
    for i = 1:length(pilot_sc)
        Xc(sc2idx(pilot_sc(i))) = pilot_sym(i);
    end

    for i = 1:length(data_sc)
        Xc(sc2idx(data_sc(i))) = data_sym(i);
    end

    % IFFT and adding CP
    a = ifft(ifftshift(Xc));
    x_cp = [a(end-cp_size+1:end); a];
end

% gen_ofdm_data
% Usage:
%   [ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, qam_num)
% Input:
%   num_ofdm_symbols : number of OFDM symbols to generate
%   qam_num : order of QAM
% Output:
%   ofdm_data     : concatenated time-domain OFDM symbols with cyclic prefix
%   tx_bits       : transmitted bits on data subcarriers
%   tx_data_syms  : transmitted 4-QAM symbols on data subcarriers
%   pilot_syms    : pilot symbols on pilot subcarriers
function [ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, qam_num)

% 802.11a/g subcarrier allocation
    active_sc  = [-26:-1 1:26];           % 52 active subcarriers
    pilot_sc = [-21 -7 7 21];           % 4 pilot tones
    data_sc  = setdiff(active_sc, pilot_sc); % 48 data tones
    
    FFT_size = 64;
    cp_size = 16;

    num_data = length(data_sc);
    num_pilot = length(pilot_sc);

    tx_bits = zeros(log2(qam_num)*num_data, num_ofdm_symbols);
    tx_data_syms = zeros(num_data, num_ofdm_symbols);
    pilot_syms = zeros(num_pilot, num_ofdm_symbols);

    ofdm_data = zeros((FFT_size + cp_size) * num_ofdm_symbols, 1);

    for sym_idx = 1:num_ofdm_symbols
        [x_cp, data_bits, data_sym, pilot_sym] = gen_ofdm_symbol(qam_num);

        start_idx = (sym_idx - 1) * (FFT_size + cp_size) + 1;
        end_idx = sym_idx * (FFT_size + cp_size);

        ofdm_data(start_idx:end_idx) = x_cp;

        tx_bits(:, sym_idx) = data_bits;
        tx_data_syms(:, sym_idx) = data_sym;
        pilot_syms(:, sym_idx) = pilot_sym;
    end
end


% plot_td_signal
% Usage:
%   plot_td_signal(x, fs, fig_title, plot_mode, region_names, region_ranges)
% Input:
%   x             : input signal in time domain
%   fs            : sampling rate in Hz
%   fig_title     : figure title
%   plot_mode     : 'Real', 'Imag', or 'Abs'
%   region_names  : 1xN cell array, label names of marked regions
%   region_ranges : Nx2 matrix, each row = [start_idx end_idx]
% Output:
%   None
%   This function plots a time-domain signal in microseconds and optionally
%   marks specified regions with vertical boundaries and labels
function plot_td_signal(x, fs, fig_title, plot_mode, region_names, region_ranges)
    t_us = (0:length(x)-1).' / fs * 1e6;

    switch lower(plot_mode)
        case 'real'
            y = real(x);
            y_label = 'Real Part';
        case 'imag'
            y = imag(x);
            y_label = 'Imaginary Part';
        case 'abs'
            y = abs(x);
            y_label = 'Magnitude';
        otherwise
            error('plot_mode must be ''Real'', ''Imag'', or ''Abs''.');
    end

    figure;
    plot(t_us, y, 'LineWidth', 1);
    grid on;
    xlabel('Time (\mus)');
    ylabel(y_label);
    title(fig_title);
    hold on;

    if nargin < 5 || isempty(region_names) || isempty(region_ranges)
        return;
    end

    yl = ylim;

    for k = 1:size(region_ranges, 1)
        idx_start = region_ranges(k, 1);
        idx_end = region_ranges(k, 2);

        xline(t_us(idx_start), '--k');
        xline(t_us(idx_end), '--k');

        text(mean(t_us(idx_start:idx_end)), yl(2) * (0.9 - 0.1*(k-1)), region_names{k}, ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
end

function rx_lts = extractLTS(rx_frame, first_lts_start, which_lts, FFT_size)
% extractLTS
% 從 rx_frame 中取出指定的 Long Training Symbol。
%
% input:
% rx_frame        : 接收端完整 frame
% first_lts_start : 第一個 LTS body 的起始 index
% which_lts       : 要取第幾個 LTS，1 表示第一個，2 表示第二個
% FFT_size        : LTS 長度，通常等於 FFT size
%
% output:
% rx_lts          : 取出的 time-domain LTS，長度為 FFT_size

    if which_lts == 1
        start_idx = first_lts_start;
    elseif which_lts == 2
        start_idx = first_lts_start + FFT_size;
    else
        error('which_lts must be 1 or 2');
    end

    end_idx = start_idx + FFT_size - 1;

    if end_idx > length(rx_frame)
        error('LTS range exceeds rx_frame length');
    end

    rx_lts = rx_frame(start_idx:end_idx);
end

function H = estimateChannelFromLTS(rx_lts, lts_f_known)
% estimateChannelFromLTS
% 使用收到的 LTS 和已知的 LTS pattern 估測 channel。
%
% 原理：
% 頻域中接收訊號可以寫成
% Y = H * X
% 所以 channel estimate 為
% H = Y / X
%
% input:
% rx_lts      : time-domain received LTS
% lts_f_known : frequency-domain known LTS
%
% output:
% H           : frequency-domain channel estimate

    if length(rx_lts) ~= length(lts_f_known)
        error('rx_lts and lts_f_known must have the same length');
    end

    % 將收到的 LTS 轉到 frequency domain
    rx_lts_f = fftshift(fft(rx_lts));

    % 避免除以 0，只在 known LTS 非零的子載波上做 channel estimation
    H = zeros(size(rx_lts_f));
    valid_idx = abs(lts_f_known) > 1e-12;

    H(valid_idx) = rx_lts_f(valid_idx) ./ lts_f_known(valid_idx);
end


function rx_symbols = extractOFDMSymbols(rx_frame, data_start, FFT_size, cp_size, num_sym)
% extractOFDMSymbols
% 從 rx_frame 中取出多個 OFDM data symbols。
%
% 注意：這裡取出的每個 OFDM symbol 都「包含 CP」。
%
% input:
% rx_frame   : 接收端完整 frame
% data_start : 第一個 data OFDM symbol 的起始 index，包含 CP
% FFT_size   : FFT size
% cp_size    : cyclic prefix 長度
% num_sym    : 要取出的 OFDM symbol 數量
%
% output:
% rx_symbols : 每一欄是一個 OFDM symbol，大小為
%              (FFT_size + cp_size) x num_sym

    sym_len = FFT_size + cp_size;
    rx_symbols = zeros(sym_len, num_sym);

    for k = 1:num_sym
        start_idx = data_start + (k-1)*sym_len;
        end_idx   = start_idx + sym_len - 1;

        if end_idx > length(rx_frame)
            fprintf('WARNING!! OFDM symbol range exceeds rx_frame length, padding zeros\n');
        
            valid_len = length(rx_frame) - start_idx + 1;
        
            if valid_len > 0
                rx_symbols(1:valid_len, k) = rx_frame(start_idx:end);
            end
        
        else
            rx_symbols(:, k) = rx_frame(start_idx:end_idx);
        end
    end
end


function Y = ofdmDemodSymbol(rx_symbol_with_cp, FFT_size, cp_size)
% ofdmDemodSymbol
% 對單一 OFDM symbol 做 demodulation。
%
% 步驟：
% 1. 移除 CP
% 2. 做 FFT
% 3. fftshift，讓子載波順序變成 -32 到 31
%
% input:
% rx_symbol_with_cp : 含 CP 的 time-domain OFDM symbol
% FFT_size          : FFT size
% cp_size           : cyclic prefix 長度
%
% output:
% Y                 : frequency-domain received OFDM symbol

    if length(rx_symbol_with_cp) ~= (FFT_size + cp_size)
        error('Input symbol length must be FFT_size + cp_size');
    end

    % 移除 cyclic prefix
    rx_no_cp = rx_symbol_with_cp(cp_size+1:end);

    % 轉到 frequency domain
    Y = fftshift(fft(rx_no_cp, FFT_size));
end


function x_hat = equalizeSymbol(Y, H)
% equalizeSymbol
% 使用 channel estimate 對 received OFDM symbol 做 equalization。
%
% 原理：
% Y = H * X
% 所以 X 的估計值為
% x_hat = Y / H
%
% input:
% Y : received frequency-domain OFDM symbol
% H : estimated channel
%
% output:
% x_hat : equalized frequency-domain symbol

    if length(Y) ~= length(H)
        error('Y and H must have the same length');
    end

    x_hat = zeros(size(Y));

    %避免除以太小的 H，造成數值爆掉
    valid_idx = abs(H) > 0.01;
    x_hat(valid_idx) = Y(valid_idx) ./ H(valid_idx);

    % x_hat = Y ./ H;
end


function [snr_dB, sig_pow, noise_pow] = calcFrameSNR(rx_frame, data_start, FFT_size, cp_size, num_sym, zero_pad_1, zero_pad_2)
% calcFrameSNR
% 估計接收 frame 的 SNR。
%
% 方法：
% 1. data symbol 區域拿來估計 signal power
% 2. 前後 zero padding 區域拿來估計 noise power
% 3. SNR = 10 log10(signal power / noise power)
%
% input:
% rx_frame   : 接收端完整 frame
% data_start : data symbols 起始位置
% FFT_size   : FFT size
% cp_size    : cyclic prefix 長度
% num_sym    : data OFDM symbol 數量
% zero_pad_1 : 前面 zero padding 區間 [start, end]
% zero_pad_2 : 後面 zero padding 區間 [start, end]
%
% output:
% snr_dB     : SNR，單位 dB
% sig_pow    : signal power
% noise_pow  : noise power

    sym_len = FFT_size + cp_size;

    % data signal 區間
    sig_start = data_start;
    sig_end   = data_start + num_sym*sym_len - 1;

    if sig_end > length(rx_frame)
        error('Signal region exceeds rx_frame length');
    end

    if zero_pad_1(2) > length(rx_frame) || zero_pad_2(2) > length(rx_frame)
        error('Zero-padding region exceeds rx_frame length');
    end

    % 取出 signal region 和 noise region
    sig_region = rx_frame(sig_start:sig_end);
    noise_region = [rx_frame(zero_pad_1(1):zero_pad_1(2)); ...
                    rx_frame(zero_pad_2(1):zero_pad_2(2))];

    % 計算平均功率
    sig_pow = mean(abs(sig_region).^2);
    noise_pow = mean(abs(noise_region).^2);

    % 換成 dB
    snr_dB = 10 * log10(sig_pow / noise_pow);
end


function plotChannelCompare(H1, H2)
% plotChannelCompare
% 比較使用第一個 LTS 和第二個 LTS 估測出的 channel。
%
% 圖一：channel magnitude
% 圖二：channel phase
%
% 題目要求觀察 subcarrier -26 到 26。

    used_subc = -26:26;

    % fftshift 後 index 對應：
    % subcarrier 0 對應 index 33
    idx = used_subc + 33;

    figure;

    % 比較 magnitude
    subplot(2,1,1);
    plot(used_subc, abs(H1(idx)), '-o', 'LineWidth', 1.2); hold on;
    plot(used_subc, abs(H2(idx)), '-x', 'LineWidth', 1.2);
    xlabel('Subcarrier Index');
    ylabel('|H|');
    title('Channel Magnitude Comparison');
    legend('1st LTS', '2nd LTS');
    grid on;

    % 比較 phase
    subplot(2,1,2);
    plot(used_subc, angle(H1(idx)), '-o', 'LineWidth', 1.2); hold on;
    plot(used_subc, angle(H2(idx)), '-x', 'LineWidth', 1.2);
    xlabel('Subcarrier Index');
    ylabel('Phase (rad)');
    title('Channel Phase Comparison');
    legend('1st LTS', '2nd LTS');
    grid on;
end


function plotConstellation(sym, plot_title, ref_sym)
% plotConstellation
% 畫星座圖。
%
% input:
% sym        : 要畫的 received 或 equalized symbols
% plot_title : 圖的標題
% ref_sym    : optional，原本 transmitted symbols，可以拿來對照

    figure;

    % 畫 received / estimated symbols
    plot(real(sym), imag(sym), '.');
    hold on;

    % 如果有提供 reference symbols，就一起畫出來比較
    if nargin >= 3 && ~isempty(ref_sym)
        plot(real(ref_sym), imag(ref_sym), 'o');
        legend('Received', 'Reference');
    end

    xlabel('In-Phase');
    ylabel('Quadrature');
    title(plot_title);
    axis equal;
    grid on;
end