%% =========================================================================
% Lab 5 Part 2 Q1
% Maximum Ratio Combining: 1 TX antenna, 2 RX antennas
%
% TX: USRP-2920 / N200 / N210 / USRP2, single antenna
% RX: USRP-2901 / B210, two antennas
%
% Q1 要求：
%   Plot the time domain signals of the signals received at the two antennas
%   with the correct time unit in microseconds,
%   showing that both antennas successfully capture the whole frame.
%% =========================================================================
try
    release(radio_Tx);
catch
end

try
    release(radio_Rx);
catch
end

try
    release(radio_Tx_mrc);
catch
end

try
    release(radio_Rx_mrc);
catch
end

try
    release(radio_Tx_mrt);
catch
end

try
    release(radio_Rx_mrt);
catch
end


clear all; close all; clc;
pause(1);

%% =========================================================================
% 1. OFDM 基本參數
%% =========================================================================

FFT_size = 64;                 % OFDM FFT size
cp_size = 16;                  % cyclic prefix length
fs = 1e6;                      % Lab 5 Part 2 指定 OFDM sample rate = 1 MHz
OFDM_sr = fs;                  % 給 USRP_init_MRC 使用，跟 Part 1 命名一致

% 802.11a/g-like subcarrier allocation
active_sc = [-26:-1 1:26];     % 52 active subcarriers
pilot_sc  = [-21 -7 7 21];     % 4 pilot tones
data_sc   = setdiff(active_sc, pilot_sc);  % 48 data tones

num_ofdm_symbols = 100;        % 題目指定 100 OFDM data symbols
qam_num = 16;                  % Part 2 使用 16-QAM

% frame 前後補 zeros
% 目的：
%   1. 讓 frame 開頭/結尾在 time-domain plot 比較清楚
%   2. STS detection 找到 STS 後，可以往前扣 pad_len 找 frame_start
pad_len = 500;

%% =========================================================================
% 2. 產生 STS / LTS / OFDM data
%% =========================================================================

% 產生 preamble
sts = gen_sts();               % Short Training Sequence，用來做 synchronization
lts = gen_lts();               % Long Training Sequence，用來做 channel estimation

% normalize power
% 作業有要求 STS / LTS / OFDM symbols power 要一致
sts = sts / rms(sts);
lts = lts / rms(lts);

% 產生 100 個 16-QAM OFDM symbols
[ofdm_data, tx_bits, tx_data_syms, pilot_syms] = ...
    gen_ofdm_data(num_ofdm_symbols, qam_num);

% normalize OFDM payload power
ofdm_data = ofdm_data / rms(ofdm_data);

% 組成完整 transmit frame
% frame 結構：
%   zeros | STS | LTS | 100 OFDM symbols | zeros
tx_frame = [
    zeros(pad_len, 1);
    sts;
    lts;
    ofdm_data;
    zeros(pad_len, 1)
];

% normalize peak，避免 USRP input 太大造成 clipping
tx_frame = tx_frame / max(abs(tx_frame));

%% =========================================================================
% 3. 畫出 TX frame，確認 frame 結構
%% =========================================================================

region_names = {'STS', 'LTS', '100 OFDM Symbols'};

region_ranges = [
    pad_len + 1, ...
    pad_len + length(sts);

    pad_len + length(sts) + 1, ...
    pad_len + length(sts) + length(lts);

    pad_len + length(sts) + length(lts) + 1, ...
    pad_len + length(sts) + length(lts) + length(ofdm_data)
];

plot_td_signal(tx_frame, fs, ...
    'Part 2 Q1: Generated TX Frame', ...
    'Abs', region_names, region_ranges);

%% =========================================================================
% 4. USRP / RF 參數
%% =========================================================================

% 使用你 Part 1 成功的 carrier frequency
fc = 885e6;

% gain 先沿用 Part 1，之後可依現場狀況調整
% 如果訊號太小：可以提高 tx_gain / rx_gain
% 如果 overflow 或 constellation 爛掉：可以降低 gain
tx_gain = 10;
rx_gain = 20;

% USRP-2920 / N200 使用 Ethernet IP
ip_2920 = '192.168.10.2';

% USRP-2901 / B210 使用 USB SerialNum
% 這裡先沿用你 Part 1 的 serial number
serial_2901 = '34D9DC3';

% master clock
% N200 / USRP-2920 固定 100 MHz
% B210 / USRP-2901 可以設 20 MHz
master_clock_rate_2920 = 100e6;
master_clock_rate_2901 = 20e6;

%% =========================================================================
% 5. 重複傳送 frame，並設定 RX capture 長度
%% =========================================================================

% 因為 TX/RX 是兩台不同 USRP，開始傳和開始收的時間不一定對齊
% 所以把同一個 frame 重複傳很多次，RX capture 長一點
% 只要中間有一包完整 frame 被收到，就可以用 STS correlation 找出來
N_repeat = 10;

tx_waveform = repmat(tx_frame, N_repeat, 1);

frame_len = length(tx_frame);

% RX capture 長度比 tx_waveform 再長一點
rx_length = (N_repeat + 2) * frame_len;

fprintf('frame_len = %d samples\n', frame_len);
fprintf('tx_waveform length = %d samples\n', length(tx_waveform));
fprintf('rx_length = %d samples\n', rx_length);

%% =========================================================================
% 6. 初始化 USRP：TX 單通道、RX 雙通道
%% =========================================================================
%% =========================================================================
% Clean up old USRP objects before initialization
%% =========================================================================



[radio_Tx, radio_Rx] = USRP_init_MRC( ...
    fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
    ip_2920, serial_2901, ...
    master_clock_rate_2920, master_clock_rate_2901);


%% =========================================================================
% 7. 重複嘗試傳收，直到抓到完整 frame
%% =========================================================================

% STS matched filter
% 用已知 STS 去跟 received signal 做 correlation
% correlation peak 代表 STS 出現的位置
match_filter = conj(flipud(sts));

% detection threshold
% 如果抓不到，可以降低 threshold
% 如果常抓錯，可以提高 threshold
threshold = 0.003 * length(sts) * mean(abs(sts).^2);

maxattempts = 50;
maxretries = 10;

success = 0;
retry_count = 0;

while ~success && retry_count < maxretries

    retry_count = retry_count + 1;

    for attempt = 1:maxattempts

        fprintf('\nQ1 attempt %d / %d, retry %d / %d\n', ...
            attempt, maxattempts, retry_count, maxretries);

        %% ------------------------------------------------------------
        % TX：USRP-2920 傳送 repeated frame
        %% ------------------------------------------------------------
        tx_underrun = radio_Tx(tx_waveform);

        if tx_underrun
            fprintf('TX underrun occurred.\n');
        end

        %% ------------------------------------------------------------
        % RX：USRP-2901 / B210 雙天線接收
        %% ------------------------------------------------------------
        [rx_matrix, len, rx_overflow] = step(radio_Rx);

        if rx_overflow
            fprintf('RX overflow occurred. Try next attempt.\n');
            continue;
        end

        if len == 0
            fprintf('No samples received. Try next attempt.\n');
            continue;
        end

        % 只保留有效 samples
        rx_matrix = rx_matrix(1:len, :);

        % 確認 RX 有回傳兩個 channel
        if size(rx_matrix, 2) < 2
            error('RX did not return two channels. Check ChannelMapping = [1 2] in USRP_init_MRC().');
        end

        rx_ant1 = rx_matrix(:, 1);
        rx_ant2 = rx_matrix(:, 2);

        %% ------------------------------------------------------------
        % 用 RX antenna 1 做 STS correlation 找 frame
        %% ------------------------------------------------------------
        corr1 = abs(conv(rx_ant1, match_filter));

        max_corr1 = max(corr1);

        fprintf('max corr antenna 1 = %.6f\n', max_corr1);
        fprintf('rx_ant1 max abs = %.4f, rms = %.4f\n', ...
            max(abs(rx_ant1)), rms(rx_ant1));
        fprintf('rx_ant2 max abs = %.4f, rms = %.4f\n', ...
            max(abs(rx_ant2)), rms(rx_ant2));

        %% ------------------------------------------------------------
        % 如果 correlation peak 夠大，嘗試切出完整 frame
        %% ------------------------------------------------------------
        if max_corr1 >= threshold

            [~, peak_idx] = max(corr1);

            % full convolution 的 peak 大約對應 STS 結束位置
            % STS 長度是 length(sts)，所以 sts_start 約為：
            sts_start = peak_idx - length(sts) + 1;

            % TX frame 前面有 pad_len zeros
            frame_start = sts_start - pad_len;

            % frame end 由 frame length 決定
            frame_end = frame_start + frame_len - 1;

            fprintf('Detected sts_start = %d\n', sts_start);
            fprintf('Detected frame_start = %d\n', frame_start);
            fprintf('Detected frame_end = %d\n', frame_end);

            % 檢查 frame 是否完整落在 received buffer 裡
            if frame_start < 1 || frame_end > length(rx_ant1)
                fprintf('Detected frame is incomplete. Try next attempt.\n');
                continue;
            end

            success = 1;
            break;
        end
    end

    %% ------------------------------------------------------------
    % 如果這輪 retry 都失敗，重開 USRP object
    %% ------------------------------------------------------------
    if success
        fprintf('\nQ1 success: complete frame detected.\n');
    else
        fprintf('\nToo many failed attempts. Restarting USRP objects...\n');

        release(radio_Tx);
        release(radio_Rx);

        [radio_Tx, radio_Rx] = USRP_init_MRC( ...
            fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
            ip_2920, serial_2901, ...
            master_clock_rate_2920, master_clock_rate_2901);
    end
end

% 用完硬體後 release
release(radio_Tx);
release(radio_Rx);

if ~success
    error('Q1 failed: cannot detect a complete frame. Try increasing gain or lowering threshold.');
end

%% =========================================================================
% 8. 根據偵測到的位置，切出兩根 RX antenna 的完整 frame
%% =========================================================================

% 因為 antenna 1 和 antenna 2 是同一台 USRP-2901 同時接收，
% 所以可以用同一組 frame_start / frame_end 切出兩路 frame
rx_frame_ant1 = rx_ant1(frame_start:frame_end);
rx_frame_ant2 = rx_ant2(frame_start:frame_end);

% Remove DC offset separately for two RX antennas
rx_frame_ant1 = rx_frame_ant1 - mean(rx_frame_ant1);
rx_frame_ant2 = rx_frame_ant2 - mean(rx_frame_ant2);

fprintf('\nExtracted RX frame length antenna 1 = %d samples\n', length(rx_frame_ant1));
fprintf('Extracted RX frame length antenna 2 = %d samples\n', length(rx_frame_ant2));

%% =========================================================================
% 9. Q1 required plots：畫兩根 RX antenna 的 time-domain signal
%% =========================================================================

% region_ranges 是以切出後的 rx_frame 為基準
% 因為 rx_frame 結構和 tx_frame 一樣：
%   zeros | STS | LTS | OFDM data | zeros
region_names_rx = {'STS', 'LTS', '100 OFDM Symbols'};

region_ranges_rx = [
    pad_len + 1, ...
    pad_len + length(sts);

    pad_len + length(sts) + 1, ...
    pad_len + length(sts) + length(lts);

    pad_len + length(sts) + length(lts) + 1, ...
    pad_len + length(sts) + length(lts) + length(ofdm_data)
];

plot_td_signal(rx_frame_ant1, fs, ...
    'Part 2 Q1: RX Antenna 1 Time-Domain Signal', ...
    'Abs', region_names_rx, region_ranges_rx);

plot_td_signal(rx_frame_ant2, fs, ...
    'Part 2 Q1: RX Antenna 2 Time-Domain Signal', ...
    'Abs', region_names_rx, region_ranges_rx);

fprintf('\nPart 2 Q1 finished.\n');
fprintf('Two received time-domain signals are plotted with time unit in microseconds.\n');
%% =========================================================================
% Lab 5 Part 2 Q2
% Process two received signals separately:
%   1. CFO correction
%   2. Channel estimation
%   3. Calculate SNR per subcarrier
%   4. Plot SNR per subcarrier of RX antenna 1 and RX antenna 2
%
% 注意：
%   這段 code 要接在 Q1 後面。
%   Q1 已經產生：
%       rx_frame_ant1
%       rx_frame_ant2
%       tx_bits
%       tx_data_syms
%       pilot_syms
%       sts
%       lts
%       lts_f_known
%       data_sc
%       pilot_sc
%% =========================================================================

fprintf('\n================ Part 2 Q2 ================\n');
fprintf('Processing RX antenna 1 and RX antenna 2 separately...\n');

%% -------------------------------------------------------------------------
% 1. Known LTS in frequency domain
%% -------------------------------------------------------------------------
% gen_lts() 的 lts 結構：
%   lts = [32-sample CP; 64-sample LTS; 64-sample LTS]
%
% 所以 lts(33:96) 是第一個 64-sample LTS body。
% 這個 lts_f_known 會拿來做 channel estimation。
lts_f_known = fftshift(fft(lts(33:96)));

%% -------------------------------------------------------------------------
% 2. 對 RX antenna 1 做 Part 1 receiver processing
%% -------------------------------------------------------------------------
% processOneRxAntenna() 會做：
%   1. 用 STS 估 CFO
%   2. CFO correction
%   3. 用 LTS 估 channel
%   4. extract OFDM symbols
%   5. equalization
%   6. pilot-assisted phase correction
%   7. demodulation
%   8. BER / SNR calculation
%
% Q2 題目要求至少要做到 CFO correction + channel estimation + SNR per subcarrier。
% 這裡直接完整跑完，後面 Q3 也可以沿用 result_ant1 / result_ant2。
use_pilot_correction = true;

fprintf('\nCheck RX frames before Q2:\n');
fprintf('rx_frame_ant1 max = %.4f, rms = %.4f\n', max(abs(rx_frame_ant1)), rms(rx_frame_ant1));
fprintf('rx_frame_ant2 max = %.4f, rms = %.4f\n', max(abs(rx_frame_ant2)), rms(rx_frame_ant2));

fprintf('Any NaN in rx_frame_ant1? %d\n', any(~isfinite(rx_frame_ant1)));
fprintf('Any NaN in rx_frame_ant2? %d\n', any(~isfinite(rx_frame_ant2)));

result_ant1 = processOneRxAntenna( ...
    rx_frame_ant1, ...          % RX antenna 1 收到的完整 frame
    tx_bits, ...                % transmitted bits
    tx_data_syms, ...           % transmitted QAM symbols
    qam_num, ...                % 16-QAM
    FFT_size, ...               % FFT size = 64
    cp_size, ...                % CP size = 16
    fs, ...                     % sample rate = 1 MHz
    pad_len, ...                % zero padding length
    sts, ...                    % STS
    lts, ...                    % LTS
    data_sc, ...                % data subcarriers
    pilot_sc, ...               % pilot subcarriers
    pilot_syms, ...             % transmitted pilot symbols
    lts_f_known, ...            % known LTS in frequency domain
    use_pilot_correction);      % 是否使用 pilot correction

%% -------------------------------------------------------------------------
% 3. 對 RX antenna 2 做同樣處理
%% -------------------------------------------------------------------------
result_ant2 = processOneRxAntenna( ...
    rx_frame_ant2, ...
    tx_bits, ...
    tx_data_syms, ...
    qam_num, ...
    FFT_size, ...
    cp_size, ...
    fs, ...
    pad_len, ...
    sts, ...
    lts, ...
    data_sc, ...
    pilot_sc, ...
    pilot_syms, ...
    lts_f_known, ...
    use_pilot_correction);

%% -------------------------------------------------------------------------
% 4. 印出兩根 antenna 的 CFO / BER / frame SNR
%% -------------------------------------------------------------------------
fprintf('\nQ2 result summary:\n');

fprintf('RX antenna 1 estimated CFO = %.2f Hz\n', result_ant1.cfo_hat);
fprintf('RX antenna 2 estimated CFO = %.2f Hz\n', result_ant2.cfo_hat);

fprintf('RX antenna 1 BER = %.6f\n', result_ant1.ber);
fprintf('RX antenna 2 BER = %.6f\n', result_ant2.ber);

fprintf('RX antenna 1 frame SNR = %.2f dB\n', result_ant1.frame_snr_dB);
fprintf('RX antenna 2 frame SNR = %.2f dB\n', result_ant2.frame_snr_dB);

%% -------------------------------------------------------------------------
% 5. 畫兩根 RX antennas 的 estimated channel magnitude / phase
%% -------------------------------------------------------------------------
% 這不是 Q2 必須的圖，但很有幫助。
% 可以檢查兩根天線的 channel response 是否不同。
subcarrier_axis = (-FFT_size/2):(FFT_size/2-1);

valid_lts_idx = abs(lts_f_known) > 1e-12;

H1_plot = result_ant1.H;
H2_plot = result_ant2.H;

H1_mag = abs(H1_plot);
H2_mag = abs(H2_plot);

H1_phase = angle(H1_plot);
H2_phase = angle(H2_plot);

% null subcarriers 不畫，設成 NaN
H1_mag(~valid_lts_idx) = NaN;
H2_mag(~valid_lts_idx) = NaN;

H1_phase(~valid_lts_idx) = NaN;
H2_phase(~valid_lts_idx) = NaN;

figure;

subplot(2,1,1);
plot(subcarrier_axis, H1_mag, '-o', 'LineWidth', 1.2);
hold on;
plot(subcarrier_axis, H2_mag, '-s', 'LineWidth', 1.2);
grid on;
xlabel('Subcarrier Index');
ylabel('|H[k]|');
title('Part 2 Q2: Estimated Channel Magnitude of Two RX Antennas');
legend('RX Antenna 1', 'RX Antenna 2', 'Location', 'best');

subplot(2,1,2);
plot(subcarrier_axis, unwrap(H1_phase), '-o', 'LineWidth', 1.2);
hold on;
plot(subcarrier_axis, unwrap(H2_phase), '-s', 'LineWidth', 1.2);
grid on;
xlabel('Subcarrier Index');
ylabel('Phase of H[k] (rad)');
title('Part 2 Q2: Estimated Channel Phase of Two RX Antennas');
legend('RX Antenna 1', 'RX Antenna 2', 'Location', 'best');

%% -------------------------------------------------------------------------
% 6. Q2 required plot：SNR per subcarrier comparison
%% -------------------------------------------------------------------------
% result_ant1.snr_per_sc_dB 和 result_ant2.snr_per_sc_dB
% 是用 equalized symbols 和 transmitted symbols 估出來的 EVM-like SNR。
%
% x-axis 用 data_sc，因為題目要看 data subcarriers 上的 SNR。
figure;

plot(data_sc, result_ant1.snr_per_sc_dB, '-o', 'LineWidth', 1.2);
hold on;

plot(data_sc, result_ant2.snr_per_sc_dB, '-s', 'LineWidth', 1.2);

grid on;
xlabel('Subcarrier Index');
ylabel('SNR per Subcarrier (dB)');
title('Part 2 Q2: SNR per Subcarrier of RX Antenna 1 and RX Antenna 2');
legend('RX Antenna 1', 'RX Antenna 2', 'Location', 'best');

fprintf('\nPart 2 Q2 finished.\n');
fprintf('SNR per subcarrier of RX antenna 1 and RX antenna 2 is plotted.\n');
%% =========================================================================
% Lab 5 Part 2 Q3
% Plot the equalized received constellations of RX antenna 1 and RX antenna 2
%
% 這題接在 Q2 後面。
% Q2 已經算出：
%   result_ant1.eq_data_syms
%   result_ant2.eq_data_syms
%
% 這裡只需要把兩根天線 equalized 後的 data tones 畫成 constellation。
%% =========================================================================

fprintf('\n================ Part 2 Q3 ================\n');

%% -------------------------------------------------------------------------
% 1. Plot equalized constellation of RX antenna 1
%% -------------------------------------------------------------------------
% result_ant1.eq_data_syms：
%   size = length(data_sc) x num_ofdm_symbols
%   每一個元素都是 equalization + pilot correction 後的 data symbol
%
% tx_data_syms(:)：
%   transmitted reference symbols
%   用來在圖上標出理想 16-QAM constellation 點
plotConstellation(result_ant1.eq_data_syms, ...
    'Part 2 Q3: Equalized Constellation - RX Antenna 1', ...
    tx_data_syms(:));

%% -------------------------------------------------------------------------
% 2. Plot equalized constellation of RX antenna 2
%% -------------------------------------------------------------------------
plotConstellation(result_ant2.eq_data_syms, ...
    'Part 2 Q3: Equalized Constellation - RX Antenna 2', ...
    tx_data_syms(:));

%% -------------------------------------------------------------------------
% 3. Print BER / SNR summary for comparison
%% -------------------------------------------------------------------------
fprintf('RX antenna 1 BER = %.6f\n', result_ant1.ber);
fprintf('RX antenna 2 BER = %.6f\n', result_ant2.ber);

fprintf('RX antenna 1 frame SNR = %.2f dB\n', result_ant1.frame_snr_dB);
fprintf('RX antenna 2 frame SNR = %.2f dB\n', result_ant2.frame_snr_dB);

%% -------------------------------------------------------------------------
% 4. Optional: Plot both constellations on the same figure for quick comparison
%% -------------------------------------------------------------------------
% 這張不是題目必須，但很方便比較兩根天線誰比較集中。
figure;
plot(real(result_ant1.eq_data_syms(:)), imag(result_ant1.eq_data_syms(:)), '.');
hold on;
plot(real(result_ant2.eq_data_syms(:)), imag(result_ant2.eq_data_syms(:)), '.');
plot(real(tx_data_syms(:)), imag(tx_data_syms(:)), 'o');

grid on;
axis equal;
xlabel('In-Phase');
ylabel('Quadrature');
title('Part 2 Q3: Constellation Comparison of RX Antenna 1 and RX Antenna 2');
legend('RX Antenna 1', 'RX Antenna 2', 'Reference', 'Location', 'best');

fprintf('\nPart 2 Q3 finished.\n');

%% =========================================================================
% Lab 5 Part 2 Q4
% Maximum Ratio Combining, MRC
%
% Q4 要做：
%   1. 使用 RX antenna 1 和 RX antenna 2 的 channel estimate
%   2. 在 frequency domain 做 MRC
%   3. 計算 MRC 後的 SNR per subcarrier
%   4. 比較：
%       (i) only RX antenna 1
%       (ii) only RX antenna 2
%       (iii) MRC of both RX antennas
%% =========================================================================

fprintf('\n================ Part 2 Q4 ================\n');
fprintf('Performing Maximum Ratio Combining using RX antenna 1 and RX antenna 2...\n');

%% -------------------------------------------------------------------------
% 1. Perform MRC
%% -------------------------------------------------------------------------
% performMRC() 會使用：
%   result_ant1.H              : RX antenna 1 的 channel estimate
%   result_ant2.H              : RX antenna 2 的 channel estimate
%   result_ant1.rx_symbols     : RX antenna 1 的 CFO-corrected OFDM symbols
%   result_ant2.rx_symbols     : RX antenna 2 的 CFO-corrected OFDM symbols
%
% MRC 公式：
%   X_hat[k] =
%       (conj(H1[k])Y1[k] + conj(H2[k])Y2[k])
%       / (|H1[k]|^2 + |H2[k]|^2)
%
% 直覺：
%   channel 比較好的 antenna 給比較大的權重
%   channel 比較差的 antenna 給比較小的權重
mrc_result = performMRC( ...
    result_ant1, ...
    result_ant2, ...
    tx_bits, ...
    tx_data_syms, ...
    qam_num, ...
    data_sc, ...
    pilot_sc, ...
    pilot_syms);

%% -------------------------------------------------------------------------
% 2. Print MRC result summary
%% -------------------------------------------------------------------------
fprintf('\nQ4 MRC result summary:\n');
fprintf('RX antenna 1 frame SNR = %.2f dB, BER = %.6f\n', ...
    result_ant1.frame_snr_dB, result_ant1.ber);

fprintf('RX antenna 2 frame SNR = %.2f dB, BER = %.6f\n', ...
    result_ant2.frame_snr_dB, result_ant2.ber);

fprintf('MRC frame SNR          = %.2f dB, BER = %.6f\n', ...
    mrc_result.frame_snr_dB, mrc_result.ber);

% 平均 SNR gain，方便報告寫
avg_snr_ant1 = mean(result_ant1.snr_per_sc_dB, 'omitnan');
avg_snr_ant2 = mean(result_ant2.snr_per_sc_dB, 'omitnan');
avg_snr_mrc  = mean(mrc_result.snr_per_sc_dB, 'omitnan');

fprintf('\nAverage SNR over data subcarriers:\n');
fprintf('RX antenna 1 average SNR = %.2f dB\n', avg_snr_ant1);
fprintf('RX antenna 2 average SNR = %.2f dB\n', avg_snr_ant2);
fprintf('MRC average SNR          = %.2f dB\n', avg_snr_mrc);

fprintf('\nAverage MRC gain:\n');
fprintf('MRC gain over RX antenna 1 = %.2f dB\n', avg_snr_mrc - avg_snr_ant1);
fprintf('MRC gain over RX antenna 2 = %.2f dB\n', avg_snr_mrc - avg_snr_ant2);

%% -------------------------------------------------------------------------
% 3. Q4 required plot: SNR per subcarrier comparison
%% -------------------------------------------------------------------------
% 題目要求在同一張圖比較：
%   (i) only RX antenna 1
%   (ii) only RX antenna 2
%   (iii) MRC of both RX antennas
figure;

plot(data_sc, result_ant1.snr_per_sc_dB, '-o', 'LineWidth', 1.2);
hold on;

plot(data_sc, result_ant2.snr_per_sc_dB, '-s', 'LineWidth', 1.2);

plot(data_sc, mrc_result.snr_per_sc_dB, '-^', 'LineWidth', 1.2);

grid on;
xlabel('Subcarrier Index');
ylabel('SNR per Subcarrier (dB)');
title('Part 2 Q4: SNR Comparison - RX1 only, RX2 only, and MRC');
legend('RX Antenna 1 only', 'RX Antenna 2 only', 'MRC of Both Antennas', ...
    'Location', 'best');


fprintf('\nPart 2 Q4 finished.\n');
fprintf('SNR per subcarrier of RX1 only, RX2 only, and MRC is plotted.\n');
%% =========================================================================
% Lab 5 Part 2 Q5
% Plot the received constellation after Maximum Ratio Combining
%
% 這題接在 Q4 後面。
% Q4 已經算出：
%   mrc_result.eq_data_syms
%   mrc_result.ber
%   mrc_result.frame_snr_dB
%   mrc_result.snr_per_sc_dB
%
% Q5 要做：
%   1. 畫 MRC 後的 constellation
%   2. 跟 Q3 的 RX antenna 1 / RX antenna 2 constellation 比較
%   3. 觀察 MRC 是否讓 constellation 更集中
%% =========================================================================

fprintf('\n================ Part 2 Q5 ================\n');

%% -------------------------------------------------------------------------
% 1. Plot MRC constellation
%% -------------------------------------------------------------------------
% mrc_result.eq_data_syms:
%   size = length(data_sc) x num_ofdm_symbols
%   這是兩根 RX antennas 做 MRC 後的 equalized data symbols
%
% tx_data_syms(:):
%   transmitted reference 16-QAM symbols
%   用來在圖上標出理想 constellation 點
plotConstellation(mrc_result.eq_data_syms, ...
    'Part 2 Q5: MRC Equalized Constellation', ...
    tx_data_syms(:));

%% -------------------------------------------------------------------------
% 2. Print BER / SNR comparison
%% -------------------------------------------------------------------------
fprintf('\nQ5 constellation quality comparison:\n');

fprintf('RX antenna 1 BER = %.6f, frame SNR = %.2f dB\n', ...
    result_ant1.ber, result_ant1.frame_snr_dB);

fprintf('RX antenna 2 BER = %.6f, frame SNR = %.2f dB\n', ...
    result_ant2.ber, result_ant2.frame_snr_dB);

fprintf('MRC          BER = %.6f, frame SNR = %.2f dB\n', ...
    mrc_result.ber, mrc_result.frame_snr_dB);

fprintf('\nPart 2 Q5 finished.\n');
fprintf('MRC constellation is plotted and compared with single-antenna constellations.\n');










%% =========================================================================
%  Lab 5 Part 2 Functions
%  請全部放在 Part2.mlx 最下面
%
%  Part 2 會用到：
%  1. MRC：USRP-2920 單天線 TX，USRP-2901/B210 雙天線 RX
%  2. MRT：USRP-2901/B210 雙天線 TX，USRP-2920 單天線 RX
%% =========================================================================


%% =========================================================================
%  1. USRP_init_MRC
%% =========================================================================
function [tx_usrp, rx_usrp] = USRP_init_MRC( ...
    fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
    ip_2920, serial_2901, master_clock_rate_2920, master_clock_rate_2901)
% USRP_init_MRC
% 初始化 MRC 用的 USRP。
%
% MRC 硬體設定：
%   TX：USRP-2920 / N200 / N210 / USRP2，單天線傳送
%   RX：USRP-2901 / B210，雙天線接收
%
% 注意：
%   USRP-2920 / N200 master clock 固定是 100 MHz。
%   所以 N200 這邊不要設定 MasterClockRate。
%
% input:
%   fc                       : carrier frequency
%   tx_gain                  : TX gain
%   rx_gain                  : RX gain
%   rx_length                : RX capture length
%   OFDM_sr                  : OFDM sample rate，例如 1e6
%   ip_2920                  : USRP-2920 IP，例如 '192.168.10.2'
%   serial_2901              : USRP-2901/B210 serial number，例如 '34D9DC3'
%   master_clock_rate_2920   : 100e6
%   master_clock_rate_2901   : 20e6
%
% output:
%   tx_usrp                  : SDRuTransmitter object
%   rx_usrp                  : SDRuReceiver object with two RX channels

    inte_factor = master_clock_rate_2920 / OFDM_sr;
    deci_factor = master_clock_rate_2901 / OFDM_sr;

    fprintf('MRC TX N200 interpolation factor = %.0f\n', inte_factor);
    fprintf('MRC RX B210 decimation factor    = %.0f\n', deci_factor);

    % ------------------------------------------------------------
    % TX：USRP-2920 / N200
    % N200 master clock 固定 100 MHz，所以不要設定 MasterClockRate。
    % ------------------------------------------------------------
    tx_usrp = comm.SDRuTransmitter( ...
        'Platform',            'N200/N210/USRP2', ...
        'IPAddress',           ip_2920, ...
        'CenterFrequency',     fc, ...
        'InterpolationFactor', inte_factor, ...
        'Gain',                tx_gain);

    % ------------------------------------------------------------
    % RX：USRP-2901 / B210，雙接收通道
    % ChannelMapping = [1 2] 表示同時收兩根 RX antenna。
    % ------------------------------------------------------------
    rx_usrp = comm.SDRuReceiver( ...
        'Platform',            'B210', ...
        'SerialNum',           serial_2901, ...
        'CenterFrequency',     fc, ...
        'Gain',                rx_gain, ...
        'SamplesPerFrame',     rx_length, ...
        'MasterClockRate',     master_clock_rate_2901, ...
        'DecimationFactor',    deci_factor, ...
        'OutputDataType',      'double', ...
        'ChannelMapping',      [1 2]);
end




%% =========================================================================
%  2. USRP_init_MRT
%% =========================================================================
function [tx_usrp, rx_usrp] = USRP_init_MRT( ...
    fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
    serial_2901, ip_2920, master_clock_rate_2901, master_clock_rate_2920)
% USRP_init_MRT
% 初始化 MRT / conjugate beamforming 用的 USRP。
%
% MRT 硬體設定：
%   TX：USRP-2901 / B210，雙天線傳送
%   RX：USRP-2920 / N200，單天線接收
%
% input:
%   fc                       : carrier frequency
%   tx_gain                  : TX gain
%   rx_gain                  : RX gain
%   rx_length                : RX capture length
%   OFDM_sr                  : OFDM sample rate
%   serial_2901              : USRP-2901/B210 serial number
%   ip_2920                  : USRP-2920 IP
%   master_clock_rate_2901   : 20e6
%   master_clock_rate_2920   : 100e6
%
% output:
%   tx_usrp                  : dual-channel SDRuTransmitter
%   rx_usrp                  : single-channel SDRuReceiver

    inte_factor = master_clock_rate_2901 / OFDM_sr;
    deci_factor = master_clock_rate_2920 / OFDM_sr;

    fprintf('MRT TX B210 interpolation factor = %.0f\n', inte_factor);
    fprintf('MRT RX N200 decimation factor    = %.0f\n', deci_factor);

    % ------------------------------------------------------------
    % TX：USRP-2901 / B210，雙通道 TX
    % ChannelMapping = [1 2] 表示兩根 TX antenna 同時送。
    % ------------------------------------------------------------
    tx_usrp = comm.SDRuTransmitter( ...
        'Platform',            'B210', ...
        'SerialNum',           serial_2901, ...
        'CenterFrequency',     fc, ...
        'Gain',                tx_gain, ...
        'MasterClockRate',     master_clock_rate_2901, ...
        'InterpolationFactor', inte_factor, ...
        'ChannelMapping',      [1 2]);

    % ------------------------------------------------------------
    % RX：USRP-2920 / N200，單通道 RX
    % N200 master clock 固定 100 MHz，所以不要設定 MasterClockRate。
    % ------------------------------------------------------------
    rx_usrp = comm.SDRuReceiver( ...
        'Platform',            'N200/N210/USRP2', ...
        'IPAddress',           ip_2920, ...
        'CenterFrequency',     fc, ...
        'Gain',                rx_gain, ...
        'SamplesPerFrame',     rx_length, ...
        'DecimationFactor',    deci_factor, ...
        'OutputDataType',      'double');
end


%% =========================================================================
%  3. processOneRxAntenna
%% =========================================================================
function result = processOneRxAntenna( ...
    rx_frame, tx_bits, tx_data_syms, qam_num, ...
    FFT_size, cp_size, fs, pad_len, sts, lts, ...
    data_sc, pilot_sc, pilot_syms, lts_f_known, use_pilot_correction)
% processOneRxAntenna
% 處理單一天線收到的一個完整 OFDM frame。
%
% 這個 function 等於把 Part 1 的 receiver chain 包起來：
%   1. 用 STS 估 CFO
%   2. CFO correction
%   3. 用 LTS 估 channel
%   4. extract OFDM symbols
%   5. equalization
%   6. pilot-assisted residual phase correction
%   7. QAM demodulation
%   8. BER / SNR calculation
%
% Part 2 Q2/Q3 會對 RX antenna 1、RX antenna 2 各跑一次。

    sc2idx = @(k) k + FFT_size/2 + 1;
    data_idx = sc2idx(data_sc);
    pilot_idx = sc2idx(pilot_sc);

    num_ofdm_symbols = size(tx_data_syms, 2);

    %% ------------------------------------------------------------
    % 1. CFO estimation using STS
    %% ------------------------------------------------------------
    % STS 每 16 samples 重複一次。
    % 如果有 CFO，相隔 16 samples 的 STS 會有 phase difference。
    % 利用這個 phase difference 換算 CFO。
    D_sts = 16;
    sts_start_in_frame = pad_len + 1;

    rx_sts = rx_frame(sts_start_in_frame : sts_start_in_frame + length(sts) - 1);

    P_sts = sum(conj(rx_sts(1:end-D_sts)) .* rx_sts(1+D_sts:end));

    cfo_hat = angle(P_sts) * fs / (2*pi*D_sts);

    %% ------------------------------------------------------------
    % 2. CFO correction
    %% ------------------------------------------------------------
    % CFO correction 公式：
    %   rx_corrected[n] = rx[n] * exp(-j*2*pi*cfo_hat*n/fs)
    n = (0:length(rx_frame)-1).';

    rx_frame_cfo = rx_frame .* exp(-1j * 2*pi * cfo_hat * n / fs);

    %% ------------------------------------------------------------
    % 3. Channel estimation using LTS
    %% ------------------------------------------------------------
    % gen_lts() 的 LTS 結構是：
    %   32-sample CP + LTS1 + LTS2
    %
    % 所以第一個 LTS body 開始位置：
    %   pad_len + length(sts) + 33
    first_lts_start = pad_len + length(sts) + 33;

    rx_lts_1 = extractLTS(rx_frame_cfo, first_lts_start, 1, FFT_size);
    rx_lts_2 = extractLTS(rx_frame_cfo, first_lts_start, 2, FFT_size);

    H1 = estimateChannelFromLTS(rx_lts_1, lts_f_known);
    H2 = estimateChannelFromLTS(rx_lts_2, lts_f_known);

    % 平均兩個 LTS 估出來的 channel，降低 noise
    H = (H1 + H2) / 2;

    %% ------------------------------------------------------------
    % 4. Extract OFDM data symbols
    %% ------------------------------------------------------------
    data_start = pad_len + length(sts) + length(lts) + 1;

    rx_symbols = extractOFDMSymbols( ...
        rx_frame_cfo, data_start, FFT_size, cp_size, num_ofdm_symbols);

    %% ------------------------------------------------------------
    % 5. Equalization + pilot correction
    %% ------------------------------------------------------------
    eq_data_syms = zeros(length(data_sc), num_ofdm_symbols);

    for k = 1:num_ofdm_symbols

        % Remove CP + FFT + fftshift
        Y = ofdmDemodSymbol(rx_symbols(:, k), FFT_size, cp_size);

        % One-tap equalization: X_hat = Y / H
        X_hat = equalizeSymbol(Y, H);

        if use_pilot_correction
            % 取出 equalized pilots
            rx_pilot = X_hat(pilot_idx);

            % 取出已知 TX pilots
            tx_pilot = pilot_syms(:, k);

            % 估計 common phase error
            theta = angle(sum(rx_pilot .* conj(tx_pilot)));

            % 補 residual phase error
            X_hat = X_hat * exp(-1j * theta);
        end

        % 只取 data tones
        rx_data_syms_bn = X_hat(data_idx);

        % ------------------------------------------------------------
        % 防止 NaN / Inf 進入 qamdemod
        % ------------------------------------------------------------
        rx_data_syms_bn(~isfinite(rx_data_syms_bn)) = 0;
        
        % ------------------------------------------------------------
        % Normalize constellation power
        % 如果 power 太小，代表這一包可能沒有正確收到，避免 0/0 產生 NaN
        % ------------------------------------------------------------
        sym_power = mean(abs(rx_data_syms_bn).^2);
        
        if ~isfinite(sym_power) || sym_power < 1e-12
            warning('Symbol power too small or non-finite at OFDM symbol %d. Set normalized data to zero.', k);
            rx_data_syms = zeros(size(rx_data_syms_bn));
        else
            rx_data_syms = rx_data_syms_bn / sqrt(sym_power);
        end
        
        % 再保險一次，確保沒有 NaN / Inf
        rx_data_syms(~isfinite(rx_data_syms)) = 0;
        
        eq_data_syms(:, k) = rx_data_syms(:);
    end

    %% ------------------------------------------------------------
    % 6. QAM demodulation
    %% ------------------------------------------------------------
    rx_bits = zeros(log2(qam_num)*length(data_sc), num_ofdm_symbols);

    for k = 1:num_ofdm_symbols

    % qamdemod 不能吃 NaN / Inf，所以先清掉
    this_syms = eq_data_syms(:, k);
    this_syms(~isfinite(this_syms)) = 0;

    rx_bits_k = qamdemod(this_syms, qam_num, ...
        'OutputType', 'bit', ...
        'UnitAveragePower', true);

    rx_bits(:, k) = rx_bits_k(:);
    end

    %% ------------------------------------------------------------
    % 7. BER calculation
    %% ------------------------------------------------------------
    bit_errors = sum(rx_bits(:) ~= tx_bits(:));
    ber = bit_errors / numel(tx_bits);

    %% ------------------------------------------------------------
    % 8. SNR calculation
    %% ------------------------------------------------------------
    frame_snr_dB = calcFrameSNRFromSymbols(eq_data_syms, tx_data_syms);
    snr_per_sc_dB = calcSubcarrierSNR(eq_data_syms, tx_data_syms);

    %% ------------------------------------------------------------
    % 9. Pack results into struct
    %% ------------------------------------------------------------
    result.cfo_hat = cfo_hat;
    result.rx_frame_cfo = rx_frame_cfo;
    result.H = H;
    result.H1 = H1;
    result.H2 = H2;
    result.rx_symbols = rx_symbols;
    result.eq_data_syms = eq_data_syms;
    result.rx_bits = rx_bits;
    result.ber = ber;
    result.frame_snr_dB = frame_snr_dB;
    result.snr_per_sc_dB = snr_per_sc_dB;
end


%% =========================================================================
%  4. performMRC
%% =========================================================================
function mrc_result = performMRC( ...
    result_ant1, result_ant2, tx_bits, tx_data_syms, qam_num, ...
    data_sc, pilot_sc, pilot_syms)
% performMRC
% 使用兩根 RX antennas 做 Maximum Ratio Combining。
%
% MRC 頻域公式：
%   X_hat[k] =
%   (conj(H1[k])*Y1[k] + conj(H2[k])*Y2[k])
%   / (|H1[k]|^2 + |H2[k]|^2)
%
% 直覺：
%   channel 強的 antenna 權重大
%   channel 弱的 antenna 權重小
%
% input:
%   result_ant1 : processOneRxAntenna(rx_frame_ant1, ...) 的結果
%   result_ant2 : processOneRxAntenna(rx_frame_ant2, ...) 的結果
%
% output:
%   mrc_result.eq_data_syms
%   mrc_result.ber
%   mrc_result.frame_snr_dB
%   mrc_result.snr_per_sc_dB

    H1 = result_ant1.H;
    H2 = result_ant2.H;

    Y1_all = result_ant1.rx_symbols;
    Y2_all = result_ant2.rx_symbols;

    FFT_size = length(H1);
    cp_size = size(Y1_all,1) - FFT_size;

    sc2idx = @(k) k + FFT_size/2 + 1;
    data_idx = sc2idx(data_sc);
    pilot_idx = sc2idx(pilot_sc);

    num_ofdm_symbols = size(tx_data_syms, 2);
    eq_data_syms_mrc = zeros(length(data_sc), num_ofdm_symbols);

    for k = 1:num_ofdm_symbols

        % 兩根 antenna 各自 OFDM demodulation
        Y1 = ofdmDemodSymbol(Y1_all(:, k), FFT_size, cp_size);
        Y2 = ofdmDemodSymbol(Y2_all(:, k), FFT_size, cp_size);

        % MRC combining
        numerator = conj(H1).*Y1 + conj(H2).*Y2;
        denominator = abs(H1).^2 + abs(H2).^2 + 1e-12;

        X_mrc = numerator ./ denominator;

        % pilot-assisted residual phase correction
        rx_pilot = X_mrc(pilot_idx);
        tx_pilot = pilot_syms(:, k);

        theta = angle(sum(rx_pilot .* conj(tx_pilot)));

        X_mrc = X_mrc * exp(-1j * theta);

        % 只取 data tones
        rx_data_syms_bn = X_mrc(data_idx);

        % normalize power
        rx_data_syms = rx_data_syms_bn / sqrt(mean(abs(rx_data_syms_bn).^2));

        eq_data_syms_mrc(:, k) = rx_data_syms(:);
    end

    %% Demodulation
    rx_bits_mrc = zeros(log2(qam_num)*length(data_sc), num_ofdm_symbols);

    for k = 1:num_ofdm_symbols
        rx_bits_k = qamdemod(eq_data_syms_mrc(:, k), qam_num, ...
            'OutputType', 'bit', ...
            'UnitAveragePower', true);

        rx_bits_mrc(:, k) = rx_bits_k(:);
    end

    %% BER
    bit_errors = sum(rx_bits_mrc(:) ~= tx_bits(:));
    ber = bit_errors / numel(tx_bits);

    %% SNR
    frame_snr_dB = calcFrameSNRFromSymbols(eq_data_syms_mrc, tx_data_syms);
    snr_per_sc_dB = calcSubcarrierSNR(eq_data_syms_mrc, tx_data_syms);

    %% Pack result
    mrc_result.eq_data_syms = eq_data_syms_mrc;
    mrc_result.rx_bits = rx_bits_mrc;
    mrc_result.ber = ber;
    mrc_result.frame_snr_dB = frame_snr_dB;
    mrc_result.snr_per_sc_dB = snr_per_sc_dB;
end


%% =========================================================================
%  5. gen_MRT_training_signal
%% =========================================================================
function [tx1_training, tx2_training, training_info] = gen_MRT_training_signal(sts, lts, pad_len, gap_len)
% gen_MRT_training_signal
% 產生 MRT channel measurement 用的 training signal。
%
% 為什麼需要這個？
%   MRT 需要知道兩條 channel：
%       TX antenna 1 -> RX
%       TX antenna 2 -> RX
%
% 如果兩根 TX antennas 同時送一樣的 LTS，RX 無法分辨哪個 channel 是哪根天線。
% 所以這裡用 time-division training。
%
% 設計：
%   TX1：zeros, STS, LTS, gap, zeros, zeros
%   TX2：zeros, zeros, zeros, gap, LTS, zeros
%
% 這樣 RX 可以：
%   前面區段估 H_tx1
%   後面區段估 H_tx2

    tx1_training = [
        zeros(pad_len,1);
        sts;
        lts;
        zeros(gap_len,1);
        zeros(length(lts),1);
        zeros(pad_len,1)
    ];

    tx2_training = [
        zeros(pad_len,1);
        zeros(length(sts),1);
        zeros(length(lts),1);
        zeros(gap_len,1);
        lts;
        zeros(pad_len,1)
    ];

    % 記錄 training 區段位置，方便畫 time-domain signal 時標註
    training_info.tx1_start = pad_len + 1;
    training_info.tx1_end = pad_len + length(sts) + length(lts);

    training_info.tx2_lts_start = pad_len + length(sts) + length(lts) + gap_len + 1;
    training_info.tx2_lts_end = training_info.tx2_lts_start + length(lts) - 1;
end


%% =========================================================================
%  6. estimateTwoTxChannelsFromTraining
%% =========================================================================
function [H_tx1, H_tx2, cfo_hat, rx_corrected] = estimateTwoTxChannelsFromTraining( ...
    rx_training_frame, sts, lts, pad_len, gap_len, FFT_size, fs, lts_f_known)
% estimateTwoTxChannelsFromTraining
% 從 MRT training frame 中估計兩根 TX antennas 的 channel。
%
% input:
%   rx_training_frame : RX 收到的 training frame
%   sts               : STS
%   lts               : LTS
%   pad_len           : zero padding length
%   gap_len           : TX1 training 和 TX2 training 中間的 gap
%   FFT_size          : FFT size
%   fs                : sample rate
%   lts_f_known       : known LTS in frequency domain
%
% output:
%   H_tx1             : TX antenna 1 -> RX channel
%   H_tx2             : TX antenna 2 -> RX channel
%   cfo_hat           : estimated CFO
%   rx_corrected      : CFO corrected training frame

    sts_len = length(sts);
    lts_len = length(lts);

    %% ------------------------------------------------------------
    % TX1 LTS position
    %% ------------------------------------------------------------
    % lts 結構是：
    %   32-sample CP + LTS1 + LTS2
    %
    % 所以 TX1 LTS body 起點：
    %   pad_len + sts_len + 33
    tx1_lts_start = pad_len + sts_len + 33;

    tx1_lts1 = rx_training_frame(tx1_lts_start : tx1_lts_start + FFT_size - 1);
    tx1_lts2 = rx_training_frame(tx1_lts_start + FFT_size : tx1_lts_start + 2*FFT_size - 1);

    %% ------------------------------------------------------------
    % CFO estimation using TX1 LTS
    %% ------------------------------------------------------------
    phase_diff = angle(sum(conj(tx1_lts1) .* tx1_lts2));

    cfo_hat = phase_diff * fs / (2*pi*FFT_size);

    %% ------------------------------------------------------------
    % CFO correction for whole training frame
    %% ------------------------------------------------------------
    n = (0:length(rx_training_frame)-1).';

    rx_corrected = rx_training_frame .* exp(-1j * 2*pi * cfo_hat * n / fs);

    %% ------------------------------------------------------------
    % Estimate H_tx1 after CFO correction
    %% ------------------------------------------------------------
    tx1_lts1_cfo = rx_corrected(tx1_lts_start : tx1_lts_start + FFT_size - 1);
    tx1_lts2_cfo = rx_corrected(tx1_lts_start + FFT_size : tx1_lts_start + 2*FFT_size - 1);

    H_tx1_1 = estimateChannelFromLTS(tx1_lts1_cfo, lts_f_known);
    H_tx1_2 = estimateChannelFromLTS(tx1_lts2_cfo, lts_f_known);

    H_tx1 = (H_tx1_1 + H_tx1_2) / 2;

    %% ------------------------------------------------------------
    % Estimate H_tx2
    %% ------------------------------------------------------------
    % TX2 的 LTS 位置：
    %   pad_len + STS + LTS + gap + 32-sample CP + 1
    tx2_lts_start = pad_len + sts_len + lts_len + gap_len + 33;

    tx2_lts1_cfo = rx_corrected(tx2_lts_start : tx2_lts_start + FFT_size - 1);
    tx2_lts2_cfo = rx_corrected(tx2_lts_start + FFT_size : tx2_lts_start + 2*FFT_size - 1);

    H_tx2_1 = estimateChannelFromLTS(tx2_lts1_cfo, lts_f_known);
    H_tx2_2 = estimateChannelFromLTS(tx2_lts2_cfo, lts_f_known);

    H_tx2 = (H_tx2_1 + H_tx2_2) / 2;
end


%% =========================================================================
%  7. gen_MRT_data_frame
%% =========================================================================
function [tx1_frame, tx2_frame, weights] = gen_MRT_data_frame( ...
    tx_bits, qam_num, FFT_size, cp_size, data_sc, pilot_sc, pilot_syms, sts, lts, pad_len, H_tx1, H_tx2)
% gen_MRT_data_frame
% 根據估到的 H_tx1 和 H_tx2 產生 MRT / conjugate beamforming waveform。
%
% MRT 的想法：
%   讓兩根 TX antenna 的訊號到 RX 時同相相加，提升 received SNR。
%
% 對每個 subcarrier k：
%   h[k] = [H_tx1[k]; H_tx2[k]]
%   w[k] = conj(h[k]) / norm(h[k])
%
% 注意：
%   beamforming weight 是加在 frequency domain，也就是 IFFT 之前。
%
% 對每個 subcarrier：
%   X1[k] = w1[k] * X[k]
%   X2[k] = w2[k] * X[k]
%
% 然後 TX1、TX2 各自 IFFT + CP，變成 time-domain waveform。
%
% 因為：
%   ||w[k]||^2 = 1
% 所以兩根 TX antenna 的總功率不會比單天線更大。

    sc2idx = @(k) k + FFT_size/2 + 1;

    data_idx = sc2idx(data_sc);
    pilot_idx = sc2idx(pilot_sc);

    bits_per_symbol = log2(qam_num);
    num_data_sc = length(data_sc);

    num_ofdm_symbols = length(tx_bits(:)) / (bits_per_symbol * num_data_sc);

    %% ------------------------------------------------------------
    % bits -> QAM symbols
    %% ------------------------------------------------------------
    qam_syms = qammod(tx_bits(:), qam_num, ...
        'InputType', 'bit', ...
        'UnitAveragePower', true);

    tx_data_syms = reshape(qam_syms, num_data_sc, num_ofdm_symbols);

    %% ------------------------------------------------------------
    % Calculate MRT weights for each subcarrier
    %% ------------------------------------------------------------
    weights = zeros(2, FFT_size);

    for k = 1:FFT_size

        h = [H_tx1(k); H_tx2(k)];

        if norm(h) < 1e-12
            % 如果 channel 太小，避免除以 0
            weights(:,k) = [1; 0];
        else
            % conjugate beamforming
            weights(:,k) = conj(h) / norm(h);
        end
    end

    %% ------------------------------------------------------------
    % Generate two TX data waveforms
    %% ------------------------------------------------------------
    tx1_data = zeros(num_ofdm_symbols*(FFT_size+cp_size), 1);
    tx2_data = zeros(num_ofdm_symbols*(FFT_size+cp_size), 1);

    write_idx = 1;

    for sym_idx = 1:num_ofdm_symbols

        % 單天線原本要傳的 frequency-domain OFDM symbol
        X = zeros(FFT_size, 1);
        X(data_idx) = tx_data_syms(:, sym_idx);
        X(pilot_idx) = pilot_syms(:, sym_idx);

        % 分給 TX1 / TX2 的 frequency-domain symbols
        X1 = zeros(FFT_size, 1);
        X2 = zeros(FFT_size, 1);

        active_idx = union(data_idx(:), pilot_idx(:));

        for n = 1:length(active_idx)
            k = active_idx(n);

            % frequency-domain beamforming
            X1(k) = weights(1,k) * X(k);
            X2(k) = weights(2,k) * X(k);
        end

        % IFFT 前要 ifftshift，因為 X1/X2 是 fftshift ordering
        x1_time = ifft(ifftshift(X1), FFT_size);
        x2_time = ifft(ifftshift(X2), FFT_size);

        % 加 CP
        x1_cp = [x1_time(end-cp_size+1:end); x1_time];
        x2_cp = [x2_time(end-cp_size+1:end); x2_time];

        tx1_data(write_idx:write_idx+FFT_size+cp_size-1) = x1_cp;
        tx2_data(write_idx:write_idx+FFT_size+cp_size-1) = x2_cp;

        write_idx = write_idx + FFT_size + cp_size;
    end

    %% ------------------------------------------------------------
    % Preamble power splitting
    %% ------------------------------------------------------------
    % 為了公平，STS/LTS 也分功率到兩根 TX antennas。
    % 這樣總發射功率大約不會比單天線更大。
    preamble_scale = 1/sqrt(2);

    tx1_frame = [
        zeros(pad_len,1);
        preamble_scale * sts;
        preamble_scale * lts;
        tx1_data;
        zeros(pad_len,1)
    ];

    tx2_frame = [
        zeros(pad_len,1);
        preamble_scale * sts;
        preamble_scale * lts;
        tx2_data;
        zeros(pad_len,1)
    ];

    %% ------------------------------------------------------------
    % Normalize peak
    %% ------------------------------------------------------------
    peak_val = max([abs(tx1_frame); abs(tx2_frame)]);

    tx1_frame = tx1_frame / peak_val;
    tx2_frame = tx2_frame / peak_val;
end


%% =========================================================================
%  8. transmitDualTxReceiveSingleFrame
%% =========================================================================
function rx_frame = transmitDualTxReceiveSingleFrame( ...
    tx_waveform_dual, frame_len, rx_length, ...
    fc, tx_gain, rx_gain, OFDM_sr, serial_2901, ip_2920, ...
    master_clock_rate_2901, master_clock_rate_2920, ...
    sts, pad_len, threshold, maxattempts, maxretries, label_name)
% transmitDualTxReceiveSingleFrame
% MRT Q9/Q10 會用到。
%
% 功能：
%   B210 雙通道 TX，N200 單通道 RX。
%   然後用 STS matched filter 找出完整 received frame。
%
% input:
%   tx_waveform_dual : N x 2 matrix，兩欄分別給 TX1/TX2
%   frame_len        : 單一 frame 長度
%   rx_length        : RX capture length
%   label_name       : 顯示目前是 TX1-only / TX2-only / MRT

    [radio_Tx, radio_Rx] = USRP_init_MRT( ...
        fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
        serial_2901, ip_2920, master_clock_rate_2901, master_clock_rate_2920);

    match_filter = conj(flipud(sts));

    success = 0;
    retry_count = 0;

    while ~success && retry_count < maxretries

        retry_count = retry_count + 1;

        for attempt = 1:maxattempts

            fprintf('\n[%s] attempt %d / %d, retry %d / %d\n', ...
                label_name, attempt, maxattempts, retry_count, maxretries);

            % dual-channel TX
            tx_underrun = radio_Tx(tx_waveform_dual);

            if tx_underrun
                fprintf('[%s] TX underrun occurred.\n', label_name);
            end

            % single-channel RX
            [received_signal, len, overflow] = step(radio_Rx);

            if overflow
                fprintf('[%s] RX overflow occurred. Try next attempt.\n', label_name);
                continue;
            end

            if len == 0
                fprintf('[%s] No samples received. Try next attempt.\n', label_name);
                continue;
            end

            received_signal = received_signal(1:len);

            % STS matched filtering
            corr = abs(conv(received_signal, match_filter));
            max_corr = max(corr);

            fprintf('[%s] max corr = %.6f, rx max = %.4f, rms = %.4f\n', ...
                label_name, max_corr, max(abs(received_signal)), rms(received_signal));

            if max_corr >= threshold

                [~, peak_idx] = max(corr);

                sts_start = peak_idx - length(sts) + 1;
                frame_start = sts_start - pad_len;
                frame_end = frame_start + frame_len - 1;

                if frame_start < 1 || frame_end > length(received_signal)
                    fprintf('[%s] detected frame incomplete. Try next attempt.\n', label_name);
                    continue;
                end

                success = 1;
                break;
            end
        end

        if ~success
            fprintf('[%s] Restarting USRP...\n', label_name);

            release(radio_Tx);
            release(radio_Rx);

            [radio_Tx, radio_Rx] = USRP_init_MRT( ...
                fc, tx_gain, rx_gain, rx_length, OFDM_sr, ...
                serial_2901, ip_2920, master_clock_rate_2901, master_clock_rate_2920);
        end
    end

    release(radio_Tx);
    release(radio_Rx);

    if ~success
        error('[%s] failed: cannot detect complete frame.', label_name);
    end

    rx_frame = received_signal(frame_start:frame_end);
end


%% =========================================================================
%  9. calcFrameSNRFromSymbols
%% =========================================================================
function snr_dB = calcFrameSNRFromSymbols(eq_data_syms, tx_data_syms)
% calcFrameSNRFromSymbols
% 用 equalized symbols 和 transmitted symbols 估整個 frame 的 SNR。
%
% 這裡使用 EVM-like 方法：
%   error = received_equalized_symbol - transmitted_symbol
%
% signal_power = mean(abs(tx_symbol)^2)
% noise_power  = mean(abs(error)^2)
%
% SNR = 10 log10(signal_power / noise_power)

    rx_vec = eq_data_syms(:);
    tx_vec = tx_data_syms(:);

    min_len = min(length(rx_vec), length(tx_vec));

    rx_vec = rx_vec(1:min_len);
    tx_vec = tx_vec(1:min_len);

    error_vec = rx_vec - tx_vec;

    sig_pow = mean(abs(tx_vec).^2);
    noise_pow = mean(abs(error_vec).^2) + 1e-12;

    snr_dB = 10*log10(sig_pow / noise_pow);
end


%% =========================================================================
%  10. calcSubcarrierSNR
%% =========================================================================
function snr_per_sc_dB = calcSubcarrierSNR(eq_data_syms, tx_data_syms)
% calcSubcarrierSNR
% 對每個 data subcarrier 分別估 SNR。
%
% input:
%   eq_data_syms : equalized received symbols
%                  size = num_data_sc x num_ofdm_symbols
%
%   tx_data_syms : transmitted symbols
%                  size = num_data_sc x num_ofdm_symbols
%
% output:
%   snr_per_sc_dB : 每個 data subcarrier 的 SNR

    num_data_sc = size(eq_data_syms, 1);

    snr_per_sc_dB = zeros(num_data_sc, 1);

    for k = 1:num_data_sc

        rx_vec = eq_data_syms(k, :).';
        tx_vec = tx_data_syms(k, :).';

        err_vec = rx_vec - tx_vec;

        sig_pow = mean(abs(tx_vec).^2);
        noise_pow = mean(abs(err_vec).^2) + 1e-12;

        snr_per_sc_dB(k) = 10*log10(sig_pow / noise_pow);
    end
end


%% =========================================================================
%  11. gen_sts
%% =========================================================================
function sts = gen_sts()
% gen_sts
% 產生 802.11a/g-like Short Training Sequence。
%
% STS 用途：
%   1. frame synchronization
%   2. matched filter detection
%   3. CFO estimation
%
% 輸出：
%   sts 長度 = 160 samples
%   由 10 個 16-sample short symbols 組成

    FFT_size = 64;

    short_sc = [-24 -20 -16 -12 -8 -4 4 8 12 16 20 24];

    sts_val = sqrt(13/6) * ...
        [1+1j, -1-1j,  1+1j, -1-1j, -1-1j,  1+1j, ...
         -1-1j, -1-1j, 1+1j,  1+1j,  1+1j,  1+1j];

    sts_f = zeros(FFT_size, 1);

    sts_f(short_sc + FFT_size/2 + 1) = sts_val;

    sts_64 = ifft(ifftshift(sts_f), FFT_size);

    sts_16 = sts_64(1:16);

    sts = repmat(sts_16, 10, 1);
end


%% =========================================================================
%  12. gen_lts
%% =========================================================================
function lts = gen_lts()
% gen_lts
% 產生 802.11a/g-like Long Training Sequence。
%
% LTS 用途：
%   1. fine synchronization
%   2. CFO estimation
%   3. channel estimation
%
% 輸出結構：
%   lts = [32-sample CP; 64-sample LTS; 64-sample LTS]
%
% 總長度：
%   160 samples

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


%% =========================================================================
%  13. gen_ofdm_symbol
%% =========================================================================
function [x_cp, data_bits, data_sym, pilot_sym] = gen_ofdm_symbol(qam_num)
% gen_ofdm_symbol
% 產生一個含 CP 的 OFDM symbol。
%
% input:
%   qam_num : QAM order，例如 4 或 16
%
% output:
%   x_cp      : time-domain OFDM symbol with CP，長度 80
%   data_bits : data subcarriers 上的 transmitted bits
%   data_sym  : data subcarriers 上的 QAM symbols
%   pilot_sym : pilot subcarriers 上的 pilot symbols

    FFT_size = 64;
    cp_size = 16;

    active_sc = [-26:-1 1:26];
    pilot_sc = [-21 -7 7 21];
    data_sc = setdiff(active_sc, pilot_sc);

    %% Generate pilot symbols
    pilot_bits = randi([0 1], length(pilot_sc), 1);
    pilot_sym = 2*pilot_bits - 1;

    %% Generate data symbols
    bit = log2(qam_num);

    data_bits = randi([0 1], length(data_sc)*bit, 1);

    data_sym = qammod(data_bits, qam_num, ...
        'InputType', 'bit', ...
        'UnitAveragePower', true);

    %% Map data and pilots to subcarriers
    Xc = zeros(FFT_size, 1);

    sc2idx = @(k) k + FFT_size/2 + 1;

    for i = 1:length(pilot_sc)
        Xc(sc2idx(pilot_sc(i))) = pilot_sym(i);
    end

    for i = 1:length(data_sc)
        Xc(sc2idx(data_sc(i))) = data_sym(i);
    end

    %% IFFT and add CP
    a = ifft(ifftshift(Xc));

    x_cp = [a(end-cp_size+1:end); a];
end


%% =========================================================================
%  14. gen_ofdm_data
%% =========================================================================
function [ofdm_data, tx_bits, tx_data_syms, pilot_syms] = gen_ofdm_data(num_ofdm_symbols, qam_num)
% gen_ofdm_data
% 產生多個 OFDM data symbols，並串成一整段 time-domain OFDM payload。
%
% input:
%   num_ofdm_symbols : OFDM symbols 數量，Part 2 是 100
%   qam_num          : QAM order，Part 2 是 16
%
% output:
%   ofdm_data    : concatenated time-domain OFDM symbols with CP
%   tx_bits      : transmitted bits
%   tx_data_syms : transmitted QAM symbols，size = 48 x num_ofdm_symbols
%   pilot_syms   : pilot symbols，size = 4 x num_ofdm_symbols

    active_sc = [-26:-1 1:26];
    pilot_sc = [-21 -7 7 21];
    data_sc = setdiff(active_sc, pilot_sc);

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


%% =========================================================================
%  15. plot_td_signal
%% =========================================================================
function plot_td_signal(x, fs, fig_title, plot_mode, region_names, region_ranges)
% plot_td_signal
% 畫 time-domain signal。
%
% x-axis 使用 microseconds，符合 Part 2 Q1 題目要求。
%
% input:
%   x             : time-domain signal
%   fs            : sample rate
%   fig_title     : title
%   plot_mode     : 'Real', 'Imag', or 'Abs'
%   region_names  : cell array，例如 {'STS', 'LTS', 'Data'}
%   region_ranges : Nx2 matrix，每一列是 [start_idx, end_idx]

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

        idx_start = max(1, region_ranges(k, 1));
        idx_end = min(length(x), region_ranges(k, 2));

        xline(t_us(idx_start), '--k');
        xline(t_us(idx_end), '--k');

        text(mean(t_us(idx_start:idx_end)), yl(2) * (0.9 - 0.1*(k-1)), region_names{k}, ...
            'HorizontalAlignment', 'center', ...
            'FontWeight', 'bold');
    end
end


%% =========================================================================
%  16. extractLTS
%% =========================================================================
function rx_lts = extractLTS(rx_frame, first_lts_start, which_lts, FFT_size)
% extractLTS
% 從 rx_frame 中取出指定的 LTS body。
%
% input:
%   rx_frame        : received frame
%   first_lts_start : 第一個 LTS body 的起始 index
%   which_lts       : 1 或 2
%   FFT_size        : LTS body 長度，通常是 64
%
% output:
%   rx_lts          : 取出的 64-sample LTS

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


%% =========================================================================
%  17. estimateChannelFromLTS
%% =========================================================================
function H = estimateChannelFromLTS(rx_lts, lts_f_known)
% estimateChannelFromLTS
% 使用 received LTS 和 known LTS pattern 估 channel。
%
% 頻域模型：
%   Y[k] = H[k] X[k]
%
% 因為 LTS 是已知的，所以：
%   H[k] = Y[k] / X[k]
%
% input:
%   rx_lts      : received LTS in time domain
%   lts_f_known : known transmitted LTS in frequency domain
%
% output:
%   H           : estimated channel in frequency domain

    if length(rx_lts) ~= length(lts_f_known)
        error('rx_lts and lts_f_known must have the same length');
    end

    rx_lts_f = fftshift(fft(rx_lts));

    H = zeros(size(rx_lts_f));

    valid_idx = abs(lts_f_known) > 1e-12;

    H(valid_idx) = rx_lts_f(valid_idx) ./ lts_f_known(valid_idx);
end


%% =========================================================================
%  18. extractOFDMSymbols
%% =========================================================================
function rx_symbols = extractOFDMSymbols(rx_frame, data_start, FFT_size, cp_size, num_sym)
% extractOFDMSymbols
% 從 rx_frame 中取出多個 OFDM data symbols。
%
% 注意：
%   這裡取出的 OFDM symbol 還包含 CP。
%
% output:
%   rx_symbols 的 size = (FFT_size + cp_size) x num_sym

    sym_len = FFT_size + cp_size;

    rx_symbols = zeros(sym_len, num_sym);

    for k = 1:num_sym

        start_idx = data_start + (k-1)*sym_len;
        end_idx = start_idx + sym_len - 1;

        if end_idx > length(rx_frame)

            fprintf('WARNING: OFDM symbol range exceeds rx_frame length, padding zeros.\n');

            valid_len = length(rx_frame) - start_idx + 1;

            if valid_len > 0
                rx_symbols(1:valid_len, k) = rx_frame(start_idx:end);
            end

        else
            rx_symbols(:, k) = rx_frame(start_idx:end_idx);
        end
    end
end


%% =========================================================================
%  19. ofdmDemodSymbol
%% =========================================================================
function Y = ofdmDemodSymbol(rx_symbol_with_cp, FFT_size, cp_size)
% ofdmDemodSymbol
% 對單一 OFDM symbol 做 demodulation。
%
% 步驟：
%   1. remove CP
%   2. FFT
%   3. fftshift
%
% output:
%   Y : frequency-domain OFDM symbol，subcarrier ordering 是 -32 到 +31

    if length(rx_symbol_with_cp) ~= (FFT_size + cp_size)
        error('Input symbol length must be FFT_size + cp_size');
    end

    rx_no_cp = rx_symbol_with_cp(cp_size+1:end);

    Y = fftshift(fft(rx_no_cp, FFT_size));
end


%% =========================================================================
%  20. equalizeSymbol
%% =========================================================================
function x_hat = equalizeSymbol(Y, H)
% equalizeSymbol
% 使用 channel estimate 做 one-tap equalization。
% X_hat = Y / H

    if length(Y) ~= length(H)
        error('Y and H must have the same length');
    end

    x_hat = zeros(size(Y));

    % 清掉非有限數值
    Y(~isfinite(Y)) = 0;
    H(~isfinite(H)) = 0;

    % 避免除以太小的 H
    valid_idx = abs(H) > 1e-6;

    x_hat(valid_idx) = Y(valid_idx) ./ H(valid_idx);

    % 防止 NaN / Inf
    x_hat(~isfinite(x_hat)) = 0;
end


%% =========================================================================
%  21. plotConstellation
%% =========================================================================
function plotConstellation(sym, plot_title, ref_sym)
% plotConstellation
% 畫 constellation。
%
% input:
%   sym        : received / equalized symbols
%   plot_title : figure title
%   ref_sym    : optional，transmitted reference symbols

    figure;

    plot(real(sym(:)), imag(sym(:)), '.');
    hold on;

    if nargin >= 3 && ~isempty(ref_sym)
        plot(real(ref_sym(:)), imag(ref_sym(:)), 'o');
        legend('Received', 'Reference');
    end

    xlabel('In-Phase');
    ylabel('Quadrature');
    title(plot_title);

    axis equal;
    grid on;
end


