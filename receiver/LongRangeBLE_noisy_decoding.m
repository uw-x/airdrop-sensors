% Decode all bluetooth packets from a file recorded by the USRP
function [ packets, filtered ] = LongRangeBLE_noisy_decoding(filename, frequencyDivider, miniarg)

    M = 2;
    freqSep = 475e3;
    bitrate = 1e6;
    advdata_length = 30;
    num_starts = 3000;
    samplerate = 2e6;
    
    if (strcmp(miniarg, "MINI") == 1)
        mini = 1;
        extractedPreamble = load('extractedPreambleMini3_2e6.mat');
    elseif (strcmp(miniarg, "DEV") == 1)
        mini = 0;
        extractedPreamble = load('extractedPreamble_2e6.mat');
    elseif (strcmp(miniarg, "nrf52832") == 1)
        mini = 3;
        extractedPreamble = load('nrf52832.mat');
    elseif (strcmp(miniarg, "nrf52840a") == 1)
        mini = 4;
        extractedPreamble = load('nrf52840a.mat');
    elseif (strcmp(miniarg, "nrf52840b") == 1)
        mini = 5;
        extractedPreamble = load('nrf52840b.mat');
    else 
        mini = 2;
        extractedPreamble = load('blackPreamble.mat'); 
    end

    function [ bytes ] = bin2bytes( bin )
      %hex2bin Converts binary vector to byte vector
      bin_reshaped = (reshape(bin.', [4, length(bin)/4])).';
      dec = bin2dec(num2str(bin_reshaped)).';
      hex = dec2hex(dec, 1).';
      bytes = transpose(cellstr(reshape(hex, 2, [])'));
    end

    function [ packet_starts, correlation_values ] = find_packets(samples)
        prb = extractedPreamble.out;
%         prb = prb(1:16);
        
        prbdet = comm.PreambleDetector(prb);
        prbdet.Threshold = 1;
        [idx,detmet] = prbdet(samples);
        detmetSort = sort(detmet,'descend');
        prbdet.Threshold = detmetSort(num_starts);
        [idx,correlation_values] = prbdet(samples);
        packet_starts = idx - length(prb);
%         packet_starts = idx;
    end

    function [ packet ] = decode_packet(index, samples)
        hDemod = comm.FSKDemodulator('ModulationOrder', M, ...
          'BitOutput', true, ...
          'SymbolMapping', 'Binary', ...
          'FrequencySeparation', freqSep, ...
          'SamplesPerSymbol', samplerate/bitrate, ...
          'SymbolRate', bitrate);

        kDemod = comm.FSKDemodulator('ModulationOrder', M, ...
          'BitOutput', true, ...
          'SymbolMapping', 'Binary', ...
          'FrequencySeparation', freqSep, ...
          'SamplesPerSymbol', round(samplerate/(bitrate/frequencyDivider)), ...
          'SymbolRate', round((bitrate/frequencyDivider)));

        bits = step(hDemod, samples);
        preamble    = bits(1:8);
        access_addr = bits(9:40);
        pdu_len     = bits(41:56);
        adv_addr    = bits(57:104);
        adv_len     = bits(105:112);
        adv_data    = bits(113:113+(advdata_length*8)-1);
        
        advDataSamples = samples( (samplerate/bitrate*112 + 1) : (((samplerate/bitrate*112 + 1)+(8*advdata_length*samplerate/bitrate))-1) );
        
        advDataBits = step(kDemod, advDataSamples);
        crc         = bits(113+advdata_length*8:end);
        packet = struct('index', index, ...
          'timestamp', index / samplerate, ...
          'delta', 1, ...,
          'snr', 0, ...,
          'preamble', preamble, ...
          'access_addr', access_addr, ...
          'pdu_len', pdu_len, ...
          'adv_addr', adv_addr, ...
          'adv_len', adv_len, ...
          'adv_data', adv_data, ...
          'adv_data_voted', {{'A5'}}, ...,
          'adv_data_raw', adv_data, ...,
          'crc', crc);

        packet.index        = index;
        packet.timestamp    = index / samplerate;
        %   flip again because they get sent over the air as little endian
        packet.preamble     = fliplr(bin2bytes(fliplr(sprintf('%.0f' , preamble))));
        packet.access_addr  = fliplr(bin2bytes(fliplr(sprintf('%.0f' , access_addr))));
        packet.pdu_len      = fliplr(bin2bytes(fliplr(sprintf('%.0f' , pdu_len))));
        packet.adv_addr     = fliplr(bin2bytes(fliplr(sprintf('%.0f' , adv_addr))));
        packet.adv_len      = fliplr(bin2bytes(fliplr(sprintf('%.0f' , adv_len))));
        packet.adv_data_raw = fliplr(bin2bytes(fliplr(sprintf('%.0f' , adv_data))));
        packet.adv_data     = bin2bytes((sprintf('%.0f' , advDataBits)));
        packet.crc          = fliplr(bin2bytes(fliplr(sprintf('%.0f' , crc))));
        
        for jj=1:(240 / frequencyDivider)
            vote = 0;
            for ii=1:frequencyDivider
                if (adv_data(ii+(frequencyDivider*(jj-1))) == 1)
                    vote = vote + 1;
                end
            end
            if (vote > (frequencyDivider/2))
                adv_data_voted(jj) = 1;
            else 
                adv_data_voted(jj) = 0;
            end
        end

        mismatches = 0;
        for jj = 1:length(advDataBits)
            if (advDataBits(jj) ~= adv_data_voted(jj))
                mismatches = mismatches + 1;
            end
        end
        
%       only consider it if the number of bits is off by 1
        if (mismatches <= 1)
            packet.adv_data_voted = bin2bytes(adv_data_voted);
        end
    end

% START
    filtered= [];
    packets = [];
    samples = read_complex_binary(filename);  
%     samples = samples(1:(0.33*length(samples)));
%     samples = samples((0.33*length(samples)):(0.66*length(samples)));
%     samples = samples((0.66*length(samples)):(1*length(samples)));
%     samples = samples((0.2*length(samples)):(0.4*length(samples)));
%     samples = samples((0.4*length(samples)):(0.6*length(samples)));
%     samples = samples((0.6*length(samples)):(0.8*length(samples)));
%     samples = samples((0.8*length(samples)):(1*length(samples)));
    packet_length = 8*(1+4+2+6+31+3)*samplerate/bitrate;
    
%     for long range data collection only, supress noise from stronger packets
    interference_indices = find(abs(samples) > .05);
    samples(interference_indices) = 0;

    figure;
    [c, lags] = xcorr(samples, extractedPreamble.out);
    stem(lags, abs(c));
    xlim([0 length(samples)]);

    figure;
    hold on;
    plot(real(samples));
    hold off;
    
%   Look for packets
    [packet_starts, correlation_values] = find_packets(samples);
    
%   Filter for packets that match expected payload
    mps_i = 1;
    for i = 1:length(packet_starts)
        if (packet_starts(i) > 0)
            if ((packet_starts(i)+packet_length - 1) < length(samples))
                packet = samples(packet_starts(i):packet_starts(i)+packet_length - 1);
                if (length(packets) == 0)  
                    afterPacket = samples((packet_starts(i)+packet_length):(packet_starts(i)+packet_length)+length(packet)-1);
                end
                
%                 [ p ] = decode_packet(packet_starts(i) - 224, packet);
                [ p ] = decode_packet(packet_starts(i), packet);
                if (frequencyDivider == 30)
                    if (strcmp(p.adv_data_voted(1), 'A5') == 0)
                        if ((strcmp(p.adv_data(1), 'A1') == 1) || (strcmp(p.adv_data_voted(1), 'A1') == 1))
                            mps_i = mps_i + 1;
                            p.snr = snr(abs(packet)', abs(afterPacket)');
                            p.correlation = correlation_values(packet_starts(i));
                            packets = [packets; p];
                        end
                    end
                elseif (frequencyDivider == 15)
                    if (strcmp(p.adv_data_voted(1), 'A5') == 0)
                        if ((strcmp(p.adv_data(1), 'CC') == 1) || (strcmp(p.adv_data_voted(1), 'CC') == 1)) && ...
                           ((strcmp(p.adv_data(2), '03') == 1) || (strcmp(p.adv_data_voted(2), '03') == 1))
                            mps_i = mps_i + 1;
                            p.snr = snr(abs(packet)', abs(afterPacket)');
                            p.correlation = correlation_values(packet_starts(i));
                            packets = [packets; p];
                        end
                    end                
                elseif (frequencyDivider == 10)
                    if (strcmp(p.adv_data_voted(1), 'A5') == 0)
                        if ((strcmp(p.adv_data(1), 'E3') == 1) || (strcmp(p.adv_data_voted(1), 'E3') == 1)) && ...
                           ((strcmp(p.adv_data(2), '80') == 1) || (strcmp(p.adv_data_voted(2), '80') == 1)) && ...
                           ((strcmp(p.adv_data(3), '07') == 1) || (strcmp(p.adv_data_voted(3), '07') == 1))
                            mps_i = mps_i + 1;
                            p.snr = snr(abs(packet)', abs(afterPacket)');
                            p.correlation = correlation_values(packet_starts(i));
                            packets = [packets; p];
                        end
                    end
                    
                 elseif (frequencyDivider == 1)
                    if (strcmp(p.adv_data_voted(1), 'A5') == 0)
                        if ((strcmp(p.adv_data(1), 'FF') == 1) || (strcmp(p.adv_data_voted(1), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(2), 'FF') == 1) || (strcmp(p.adv_data_voted(2), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(3), 'FF') == 1) || (strcmp(p.adv_data_voted(3), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(4), 'FC') == 1) || (strcmp(p.adv_data_voted(4), 'FC') == 1)) && ...
                           ((strcmp(p.adv_data(5), '00') == 1) || (strcmp(p.adv_data_voted(5), '00') == 1)) && ...
                           ((strcmp(p.adv_data(6), '00') == 1) || (strcmp(p.adv_data_voted(6), '00') == 1)) && ...
                           ((strcmp(p.adv_data(7), '00') == 1) || (strcmp(p.adv_data_voted(7), '00') == 1)) && ...
                           ((strcmp(p.adv_data(8), '0F') == 1) || (strcmp(p.adv_data_voted(8), '0F') == 1)) && ...
                           ((strcmp(p.adv_data(9), 'FF') == 1) || (strcmp(p.adv_data_voted(9), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(10), 'FF') == 1) || (strcmp(p.adv_data_voted(10), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(11), 'FF') == 1) || (strcmp(p.adv_data_voted(11), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(12), 'C0') == 1) || (strcmp(p.adv_data_voted(12), 'C0') == 1)) && ...
                           ((strcmp(p.adv_data(13), '00') == 1) || (strcmp(p.adv_data_voted(13), '00') == 1)) && ...
                           ((strcmp(p.adv_data(14), '00') == 1) || (strcmp(p.adv_data_voted(14), '00') == 1)) && ...
                           ((strcmp(p.adv_data(15), '00') == 1) || (strcmp(p.adv_data_voted(15), '00') == 1)) && ...
                           ((strcmp(p.adv_data(16), '00') == 1) || (strcmp(p.adv_data_voted(16), '00') == 1)) && ...
                           ((strcmp(p.adv_data(17), '00') == 1) || (strcmp(p.adv_data_voted(17), '00') == 1)) && ...
                           ((strcmp(p.adv_data(18), '00') == 1) || (strcmp(p.adv_data_voted(18), '00') == 1)) && ...
                           ((strcmp(p.adv_data(19), '00') == 1) || (strcmp(p.adv_data_voted(19), '00') == 1)) && ...
                           ((strcmp(p.adv_data(20), '00') == 1) || (strcmp(p.adv_data_voted(20), '00') == 1)) && ...
                           ((strcmp(p.adv_data(21), '00') == 1) || (strcmp(p.adv_data_voted(21), '00') == 1)) && ...
                           ((strcmp(p.adv_data(22), '00') == 1) || (strcmp(p.adv_data_voted(22), '00') == 1)) && ...
                           ((strcmp(p.adv_data(23), '00') == 1) || (strcmp(p.adv_data_voted(23), '00') == 1)) && ...
                           ((strcmp(p.adv_data(24), '00') == 1) || (strcmp(p.adv_data_voted(24), '00') == 1)) && ...
                           ((strcmp(p.adv_data(25), '00') == 1) || (strcmp(p.adv_data_voted(25), '00') == 1)) && ...
                           ((strcmp(p.adv_data(26), '00') == 1) || (strcmp(p.adv_data_voted(26), '00') == 1)) && ...
                           ((strcmp(p.adv_data(27), '3F') == 1) || (strcmp(p.adv_data_voted(27), '3F') == 1)) && ...
                           ((strcmp(p.adv_data(28), 'FF') == 1) || (strcmp(p.adv_data_voted(28), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(29), 'FF') == 1) || (strcmp(p.adv_data_voted(29), 'FF') == 1)) && ...
                           ((strcmp(p.adv_data(30), 'FF') == 1) || (strcmp(p.adv_data_voted(30), 'FF') == 1))
                            mps_i = mps_i + 1;
                            p.snr = snr(abs(packet)', abs(afterPacket)');
                            p.correlation = correlation_values(packet_starts(i));
                            packets = [packets; p];
                        end
                    end
                end
            end
        end        
    end
    
%   Remove elements with excessively higher SNR
    f_i = 1;
    for j = 1:2
        averageSnr = 0;
        
        for i = 1:length(packets)
            averageSnr = averageSnr + packets(i).snr;
        end
        
        averageSnr = averageSnr / length(packets);
        
        for i = length(packets):-1:1
            if (packets(i).snr > 2*averageSnr)
                filtered = [filtered; packets(i)];
                f_i = f_i + 1;
                packets(i) = [];
            end
        end
    end
    
%   Calculate interframe spacing
    for i = 2:length(packets)
        packets(i).delta = packets(i).timestamp - packets(i-1).timestamp;
    end
    
%   Filter packets that have less than 6ms interframe spacing
    for i = length(packets):-1:1
        if (packets(i).delta<0.001)
            filtered = [filtered; packets(i)];
            f_i = f_i + 1;
            packets(i) = [];
        end
    end
        
%   Recalc interframe spacing        
    for i = 2:length(packets)
        packets(i).delta = packets(i).timestamp - packets(i-1).timestamp;
    end
    
    for i = 1:length(packets)
        xline(packets(i).index, ':r');
    end
    
    fprintf('\nMini %d\n', mini);
    fprintf('\nPackets detected %d\n', length(packet_starts));
    fprintf('\nMatched Payloads %d\n', mps_i-1);
    fprintf('\nTotal %d packets decoded\n', length(packets));
end
