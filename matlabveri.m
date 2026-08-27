% TERS SARKAÇ GERÇEK ZAMANLI VERİ EKRANI 

clear; clc; close all;

COM_PORT = 'COM10';
BAUD_RATE = 115200;


eski_portlar = serialportfind;
if ~isempty(eski_portlar)
    delete(eski_portlar);
end

try
    s = serialport(COM_PORT, BAUD_RATE);
    configureTerminator(s, "CR/LF"); 
catch
    error('Seri porta bağlanılamadı!');
end

pencere_genisligi = 300; 


fig = figure('Name', 'Sarkaç Analiz Paneli', 'NumberTitle', 'off', 'Position', [100, 50, 900, 900]);
t = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'Ters Sarkaç Gerçek Zamanlı Veri Akışı', 'FontWeight', 'bold', 'FontSize', 14);

% --- 1. BÖLME: KONUMLAR ---
ax1 = nexttile;
title('Konum (0-360 Derece)');
ylabel('Açı (°)');
ylim([-10 370]); 
xlim([0 pencere_genisligi]);
grid on; hold on;
line_m_konum = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Motor Konum');
line_s_konum = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Sarkaç Konum');
legend('Location', 'northeast');

% --- 2. BÖLME: HIZLAR ---
ax2 = nexttile;
title('Hız (Derece/Sn)');
ylabel('Hız (°/s)');
ylim([-800 800]); 
xlim([0 pencere_genisligi]);
grid on; hold on;
line_m_hiz = animatedline('Color', 'c', 'LineWidth', 1.5, 'DisplayName', 'Motor Hızı');
line_s_hiz = animatedline('Color', 'm', 'LineWidth', 1.5, 'DisplayName', 'Sarkaç Hızı');
legend('Location', 'northeast');

% --- 3. BÖLME: PWM ---
ax3 = nexttile;
title('Motora Uygulanan PWM');
ylabel('PWM (0-80)');
xlabel('Zaman (Ornek Sayisi)');
ylim([0 80]);
xlim([0 pencere_genisligi]);
grid on; hold on;
line_pwm = animatedline('Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'PWM');
legend('Location', 'northeast');


disp('Veri okunuyor... Durdurmak için figür penceresini kapatın.');
flush(s); 
ornek_sayisi = 0;


while isgraphics(fig)
    try
        dataStr = readline(s);
        
        if isempty(dataStr) || strlength(dataStr) < 10
            continue;
        end
        
        nums = regexp(dataStr, ':\s*([+-]?\d*\.?\d+)', 'tokens');
        
        if length(nums) >= 5
           
            if ornek_sayisi >= pencere_genisligi
                clearpoints(line_m_konum);
                clearpoints(line_s_konum);
                clearpoints(line_m_hiz);
                clearpoints(line_s_hiz);
                clearpoints(line_pwm);
                ornek_sayisi = 0; 
                drawnow; 
            end
            
            ornek_sayisi = ornek_sayisi + 1;
            
         
            val_m_konum = mod(str2double(nums{1}{1}), 360);
            val_s_konum = mod(str2double(nums{2}{1}), 360);
            val_m_hiz   = str2double(nums{3}{1});
            val_s_hiz   = str2double(nums{4}{1});
            val_pwm     = str2double(nums{5}{1});
            
           
            addpoints(line_m_konum, ornek_sayisi, val_m_konum);
            addpoints(line_s_konum, ornek_sayisi, val_s_konum);
            addpoints(line_m_hiz, ornek_sayisi, val_m_hiz);
            addpoints(line_s_hiz, ornek_sayisi, val_s_hiz);
            addpoints(line_pwm, ornek_sayisi, val_pwm);
            
           
            if mod(ornek_sayisi, 5) == 0
                drawnow limitrate;
            end
        end
    catch ME
        break;
    end
end

clear s;
disp('Bağlantı kapatıldı.');