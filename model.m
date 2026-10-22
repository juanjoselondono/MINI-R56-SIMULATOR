%% GEMELO DIGITAL DE DINÁMICA LONGITUDINAL - MINI COOPER R56 (N16)
% Autor: JARVIS
% Arquitectura: Planta Empírica y Cinemática Acoplada (Perfil Optimizado)
clear; clc; close all;

%% 1. CONSTANTES GLOBALES DE FÍSICA Y COMBUSTIÓN
env.g       = 9.81;                 % Aceleración de gravedad [m/s^2]
env.rho_air = 1.225;                % Densidad del aire estándar [kg/m^3]

% --- CONFIGURACIÓN DE PENDIENTE DE LA VÍA ---
slope_mode  = 'pct';                % Opciones: 'deg' (Grados), 'pct' (Porcentaje %), 'rad' (Radianes)
slope_value = 0;                    % Ingrese el valor según el modo elegido (0% para trazado plano)

switch lower(slope_mode)
    case 'deg'
        env.theta = deg2rad(slope_value);           % Conversión de Grados a Radianes
    case 'pct'
        env.theta = atan(slope_value / 100);        % Conversión de Porcentaje (%) a Radianes
    case 'rad'
        env.theta = slope_value;                    % Asignación directa en Radianes
    otherwise
        error('Modo de pendiente no válido. Use ''deg'', ''pct'' o ''rad''.');
end

% Termodinámica del Combustible
fuel.LHV    = 44e6;                 % Poder Calorífico Inferior [J/kg]
fuel.AFR    = 14.7;                 % Relación Estequiométrica Aire-Combustible

%% 2. MATRIZ DE ESPECIFICACIONES DEL VEHÍCULO (Motor N16)
car.M           = 1420;             % Peso en orden de marcha [kg]
car.Cd          = 0.378;            % Coeficiente de resistencia aerodinámica (Drag)
car.A           = 1.99;             % Área frontal proyectada [m^2]
car.gear_ratios = [3.214, 1.792, 1.194, 0.914, 0.784, 0.683]; % Caja manual GS6-55BG
car.final_drive = 4.353;            % Relación del diferencial final
car.eta_t       = 0.92;             % Eficiencia de la transmisión (~92%)

% Factores de masa equivalente por inercia rotacional (1ª a 6ª marcha)
car.mass_factors = [0.50, 0.20, 0.18, 0.16, 0.14, 0.11];

% Geometría del Motor
car.V_d         = 1.598e-3;         % Desplazamiento volumétrico [m^3]
car.CR          = 11.0;             % Relación de compresión

% Neumáticos y Ruedas (175/65 R15)
tire_width      = 0.175;
aspect_ratio    = 0.65;
rim_diameter    = 15 * 0.0254;
car.R_w         = (rim_diameter / 2) + (tire_width * aspect_ratio); % Radio efectivo ~0.3043 m

% --- PARÁMETROS PARA F_XMAX (TRACCIÓN DELANTERA FWD) ---
car.mu          = 0.90;      % [-] Fricción seco
car.L           = 2.467;     % [m] Batalla
car.c           = 1.529;     % [m] CG al eje trasero (Reparto ~62/38)
car.h           = 0.503;     % [m] Altura estimada del CG
car.t_f         = 1.459;     % [m] Trocha delantera
car.t_r         = 1.467;     % [m] Trocha trasera
car.K_s_f       = 28000;     % [N/m] Resorte delantero OEM
car.K_s_r       = 37100;     % [N/m] Resorte trasero OEM
car.s_f         = 1.059;     % [m] Separación resortes delanteros
car.s_r         = 1.067;     % [m] Separación resortes traseros

% Rigidez de balanceo
car.K_phi_f     = 0.5 * car.K_s_f * car.s_f^2;   % [N*m/rad]
car.K_phi_r     = 0.5 * car.K_s_r * car.s_r^2;   % [N*m/rad]
car.K_phi       = car.K_phi_f + car.K_phi_r;     % [N*m/rad]

%% 3. MODELO DE PLANTA EMPÍRICA DEL MOTOR (Datos de Dinamómetro)
car.rpm_axis    = 1000:50:6500;
car.omega       = car.rpm_axis * (2 * pi / 60); % Velocidad angular [rad/s]

% Nodos críticos extraídos de la curva empírica
rpm_raw    = [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4250, 4500, 5000, 5500, 6000, 6400, 6500];
torque_raw = [75, 120, 135, 145, 150, 155, 158, 160, 158, 155, 148, 142, 125, 118]; 

% Interpolación PCHIP del Torque 
car.T_engine = interp1(rpm_raw, torque_raw, car.rpm_axis, 'pchip');

% Cálculo acoplado de Potencia (P = T * w)
car.P_engine_W  = car.T_engine .* car.omega;     % Potencia en Vatios [W]
car.HP_engine   = car.P_engine_W / 745.7;        % Potencia en HP

%% 4. RESISTENCIAS LONGITUDINALES (AMBIENTAL Y VÍA)
v_kmh = 0.1:1:220;         % Rango de velocidad [km/h]
v_ms  = v_kmh / 3.6;       % Conversión a [m/s]
v_mph = v_kmh / 1.60934;   % Conversión a millas por hora [mph]

% ------------------------------- Drag ---------------------------------
F_drag    = 0.5 * env.rho_air * car.A * car.Cd * (v_ms.^2);

% ------------------------------- Rolling ------------------------------
F_normal  = car.M * env.g * cos(env.theta) * ones(size(v_ms));
f_r_dynamic = 0.01 .* (1 + (v_mph ./ 100));
F_rolling = f_r_dynamic .* F_normal; 

% ------------------------------- Pendiente ----------------------------
F_pend    = car.M * env.g * sin(env.theta) * ones(size(v_ms));

% --------------------------- Demanda Ambiental -----------------------
F_resistive   = F_drag + F_rolling + F_pend;
P_demanded_W  = F_resistive .* v_ms;            % Potencia requerida en Vatios [W]
P_demanded_kW = P_demanded_W / 1000;            % En kW

%% 5. MULTIPLICACIÓN DE TRANSMISIÓN Y CÁLCULOS POR MARCHA
num_gears = length(car.gear_ratios);
V_gear_kmh     = zeros(num_gears, length(car.rpm_axis));
P_available_kW = zeros(num_gears, length(car.rpm_axis));
T_wheel_Nm     = zeros(num_gears, length(car.rpm_axis));
F_tractive_N   = zeros(num_gears, length(car.rpm_axis));
a_x_ms2        = zeros(num_gears, length(car.rpm_axis));

for g = 1:num_gears
    total_ratio = car.gear_ratios(g) * car.final_drive;
    
    % Correspondencia cinemática
    w_wheel = car.omega / total_ratio;
    v_wheel_ms = w_wheel * car.R_w;
    V_gear_kmh(g, :) = v_wheel_ms * 3.6;
    
    % Potencia disponible en rueda (kW)
    P_available_kW(g, :) = (car.P_engine_W * car.eta_t) / 1000;
    
    % Torque y Fuerza Tractiva en rueda
    T_wheel_Nm(g, :)   = car.T_engine * total_ratio * car.eta_t;
    F_tractive_N(g, :) = T_wheel_Nm(g, :) / car.R_w;
    
    % Aceleración ideal sin considerar límite de tracción (solo motor - resistencias)
    v_mph_gear = V_gear_kmh(g, :) / 1.60934;
    F_drag_gear = 0.5 * env.rho_air * car.A * car.Cd * (v_wheel_ms.^2);
    F_rolling_gear = (0.01 .* (1 + (v_mph_gear ./ 100))) .* (car.M * env.g * cos(env.theta));
    F_pend_gear = car.M * env.g * sin(env.theta);
    F_res_gear = F_drag_gear + F_rolling_gear + F_pend_gear;
    
    M_equivalent = car.M * (1 + car.mass_factors(g));
    a_x_ms2(g, :) = (F_tractive_N(g, :) - F_res_gear) / M_equivalent;
end

%% 6. CÁLCULO DEDICADO DE LA FUERZA MÁXIMA DE TRACCIÓN (F_XMAX - FWD)
W_weight = car.M * env.g * cos(env.theta);    % Peso normal del vehículo [N]

% Numerador
num_Fx = car.mu * (W_weight * car.c / car.L);

% Denominador
den_Fx = 1 + (car.h / car.L)*car.mu + ((2 * car.mu * car.R_w) / (car.final_drive * car.t_f)) * (car.K_phi_r / car.K_phi);

% Fuerza máxima de tracción límite en rueda motriz [N]
car.F_x_max = num_Fx / den_Fx;

%% 7. EVALUACIÓN DE FUERZA TRACTIVA EFECTIVA Y ACELERACIÓN REAL
F_tractive_effective = zeros(num_gears, length(car.rpm_axis));
a_x_real             = zeros(num_gears, length(car.rpm_axis));

for g = 1:num_gears
    % Condicional evaluada punto a punto:
    F_tractive_effective(g, :) = min(F_tractive_N(g, :), car.F_x_max);
    
    % Recálculo dinámico de la aceleración real transmisible al suelo
    total_ratio = car.gear_ratios(g) * car.final_drive;
    w_wheel = car.omega / total_ratio;
    v_wheel_ms = w_wheel * car.R_w;
    v_mph_gear = V_gear_kmh(g, :) / 1.60934;
    
    F_drag_gear    = 0.5 * env.rho_air * car.A * car.Cd * (v_wheel_ms.^2);
    F_rolling_gear = (0.01 .* (1 + (v_mph_gear ./ 100))) .* (car.M * env.g * cos(env.theta));
    F_pend_gear    = car.M * env.g * sin(env.theta);
    F_res_gear     = F_drag_gear + F_rolling_gear + F_pend_gear;
    
    M_equivalent   = car.M * (1 + car.mass_factors(g));
    
    % Aceleración real acelerable por el auto en la vía
    a_x_real(g, :) = (F_tractive_effective(g, :) - F_res_gear) / M_equivalent;
end

%% 8. SISTEMA DE VISUALIZACIÓN GRÁFICA (PANEL PRINCIPAL - FIGURA 1)
figure('Name', 'Figura 1: Gemelo Digital - Dinámica Vehicular R56 (N16)', 'Color', [0.1 0.1 0.1], 'Position', [50 30 1300 900]);
colors = lines(num_gears);

% Subgráfico 1: Planta de Rendimiento del Motor
ax1 = subplot(3, 2, 1);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
yyaxis left
plot(car.rpm_axis, car.T_engine, 'Color', [1.0, 0.62, 0.23], 'LineWidth', 2.5);
ylabel('Torque [Nm]', 'Color', 'w', 'FontWeight', 'bold'); ylim([0 200]);
yyaxis right
plot(car.rpm_axis, car.HP_engine, '--', 'Color', [0.88, 0.42, 0.46], 'LineWidth', 2.5);
ylabel('Potencia [HP]', 'Color', 'w', 'FontWeight', 'bold'); ylim([0 200]);
xlabel('Motor [RPM]', 'Color', 'w'); title('Planta de Rendimiento del Motor', 'Color', 'w');

% Subgráfico 2: Fuerzas de Resistencia Acopladas
ax2 = subplot(3, 2, 2);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
plot(v_kmh, F_drag, 'w--', 'LineWidth', 1.5);
plot(v_kmh, F_rolling, 'b-.', 'LineWidth', 2.0); 
plot(v_kmh, F_pend, 'g:', 'LineWidth', 2.0); 
plot(v_kmh, F_resistive, 'r-', 'LineWidth', 2);
xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Fuerza [N]', 'Color', 'w');
title('Fuerzas de Resistencia Acopladas', 'Color', 'w');
legend('F_{drag}', 'F_{rolling}', 'F_{pend}', 'F_{total}', 'TextColor', 'w', 'Color', 'k', 'Location', 'northwest');

% Subgráfico 3: Mapa Cinemático
ax3 = subplot(3, 2, 3);
set(ax3, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
gear_legends = cell(num_gears, 1);
for g = 1:num_gears
    plot(V_gear_kmh(g, :), car.rpm_axis, 'Color', colors(g, :), 'LineWidth', 2.0);
    gear_legends{g} = sprintf('%d° Marcha', g);
end
xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Motor [RPM]', 'Color', 'w');
title('Mapa Cinemático: Velocidad vs RPM por Marcha', 'Color', 'w');
xlim([0 260]); ylim([1000 6500]);
legend(gear_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'southeast');

% Subgráfico 4: Torque Efectivo en Rueda
ax4 = subplot(3, 2, 4);
set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
torque_legends = cell(num_gears, 1);
for g = 1:num_gears
    plot(V_gear_kmh(g, :), T_wheel_Nm(g, :), 'Color', colors(g, :), 'LineWidth', 2.0);
    torque_legends{g} = sprintf('%d° Marcha', g);
end
xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Torque en Rueda [Nm]', 'Color', 'w');
title('Torque Efectivo en Rueda por Marcha', 'Color', 'w'); xlim([0 260]);
legend(torque_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'northeast');

% Subgráfico 5: Balance de Potencia del Vehículo
ax5 = subplot(3, 2, [5, 6]);
set(ax5, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
plot(v_kmh, P_demanded_kW, 'r-', 'LineWidth', 2.5);
power_legends = cell(num_gears + 1, 1);
power_legends{1} = 'Potencia Requerida (Ambiente)';
for g = 1:num_gears
    plot(V_gear_kmh(g, :), P_available_kW(g, :), 'Color', colors(g, :), 'LineWidth', 1.8);
    power_legends{g + 1} = sprintf('%d° Marcha', g);
end
xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Potencia [kW]', 'Color', 'w');
title('Balance de Potencia del Vehículo', 'Color', 'w'); 
ylim([0 max(P_available_kW(1, :))*1.2]); xlim([0 260]);
legend(power_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'northwest');

%% 9. SEGUNDA PESTAÑA / VENTANA: ANÁLISIS DE ACELERACIÓN Y ADHERENCIA COMPLETO
figure('Name', 'Figura 2: Capacidad, Fuerza Tractiva y Aceleración Real (con Adherencia)', 'Color', [0.1 0.1 0.1], 'Position', [100 50 1200 850]);

% --- SUBPLOT 1 (Arriba Izquierda): Aceleración Teórica sin Límite ---
ax6 = subplot(2, 2, 1);
set(ax6, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
accel_legends = cell(num_gears + 1, 1);
for g = 1:num_gears
    plot(V_gear_kmh(g, :), a_x_ms2(g, :), 'Color', colors(g, :), 'LineWidth', 1.8);
    accel_legends{g} = sprintf('%d° Marcha', g);
end

P_max_avail_W = max(car.P_engine_W) * car.eta_t;
P_excess_W     = max(0, P_max_avail_W - P_demanded_W);
F_excess       = P_excess_W ./ v_ms;
M_eq_avg       = car.M * (1 + mean(car.mass_factors));
a_x_envelope   = F_excess / M_eq_avg;

plot(v_kmh, a_x_envelope, 'm--', 'LineWidth', 2.0);
accel_legends{num_gears + 1} = 'Envolvente Máxima Neta';

yline(0, 'w--', 'LineWidth', 1.0);
xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Aceleración a_x [m/s^2]', 'Color', 'w');
title('Aceleración Teórica (Sin Límite de Adherencia)', 'Color', 'w'); 
xlim([0 260]);
max_a = max(a_x_ms2(1, :));
if max_a > 0, ylim([-0.5, max_a*1.15]); else, ylim([min(a_x_ms2(end,:))*1.15, 0.5]); end
legend(accel_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'northeast');

% --- SUBPLOT 2 (Arriba Derecha): Fuerza Tractiva vs F_xmax ---
ax7 = subplot(2, 2, 2);
set(ax7, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;
ftract_legends = cell(num_gears + 1, 1);
for g = 1:num_gears
    plot(V_gear_kmh(g, :), F_tractive_N(g, :), 'Color', colors(g, :), 'LineWidth', 1.8);
    ftract_legends{g} = sprintf('F_{trac} %d° Marcha', g);
end

yline(car.F_x_max, 'r--', 'LineWidth', 2.2);
ftract_legends{num_gears + 1} = sprintf('Límite Adherencia F_{x,max} (%.1f N)', car.F_x_max);

xlabel('Velocidad [km/h]', 'Color', 'w'); ylabel('Fuerza Tractiva F_{trac} [N]', 'Color', 'w');
title('Fuerza Tractiva vs Límite F_{x,max}', 'Color', 'w'); 
xlim([0 260]); ylim([0 max(F_tractive_N(1, :))*1.15]);
legend(ftract_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'northeast');

% --- SUBPLOT 3 (Abajo Ocupando el Ancho): Aceleración Real Transmisible (Slip) ---
ax8 = subplot(2, 2, [3, 4]);
set(ax8, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); hold on; grid on;

real_accel_legends = cell(num_gears + 1, 1);

for g = 1:num_gears
    plot(V_gear_kmh(g, :), a_x_real(g, :), 'Color', colors(g, :), 'LineWidth', 2.2);
    real_accel_legends{g} = sprintf('Acel. Real %d° Marcha', g);
end

plot(v_kmh, a_x_envelope, 'm--', 'LineWidth', 2.0);
real_accel_legends{num_gears + 1} = 'Envolvente Máxima Teórica';

yline(0, 'w--', 'LineWidth', 1.2);
xlabel('Velocidad [km/h]', 'Color', 'w'); 
ylabel('Aceleración Real a_x [m/s^2]', 'Color', 'w');
title('Aceleración Real Transmisible al Chasis (Limitada por Patinaje / Slip)', 'Color', 'w'); 
xlim([0 260]);

max_a_real = max(a_x_real(1, :));
if max_a_real > 0
    ylim([-0.5, max_a_real * 1.2]);
else
    ylim([min(a_x_real(end, :)) * 1.2, 0.5]);
end

legend(real_accel_legends, 'TextColor', 'w', 'Color', 'k', 'Location', 'northeast');

%% 10. MÓDULO PISTA (ÓVALO - ETAPA 1): CÁLCULO DE VELOCIDAD MÁXIMA EN CURVA
% --- 10.1 PARÁMETROS DE LA CURVA ---
R_curva       = 80;                % Radio de curvatura del óvalo [m]
margin_safety = 0.80;              % Factor de seguridad (80% del límite de adherencia)
F_limite_util = margin_safety * car.F_x_max; % Límite de fuerza vectorial utilizable [N]

% --- 10.2 ITERACIÓN PARA HALLAR LA VELOCIDAD MÁXIMA EN CURVA POR ADHERENCIA ---
v_search_kmh = 1:0.1:220;
v_search_ms  = v_search_kmh / 3.6;

v_curva_adherencia_ms = 0;

for i = 1:length(v_search_ms)
    v_test = v_search_ms(i);
    v_mph_test = v_search_kmh(i) / 1.60934;
    
    % Fuerza centrípeta requerida (Lateral)
    F_y = car.M * (v_test^2) / R_curva;
    
    % Fuerza tractiva longitudinal requerida a vel. constante (Resistencias)
    F_drag_c    = 0.5 * env.rho_air * car.A * car.Cd * (v_test^2);
    F_rolling_c = (0.01 * (1 + (v_mph_test / 100))) * (car.M * env.g * cos(env.theta));
    F_pend_c    = car.M * env.g * sin(env.theta);
    F_x_req     = F_drag_c + F_rolling_c + F_pend_c;
    
    % Vector Resultante por Círculo de Adherencia Combinado
    F_resultante = sqrt(F_x_req^2 + F_y^2);
    
    % Verificación de la condición del 80% de adherencia límite
    if F_resultante > F_limite_util
        if i > 1
            v_curva_adherencia_ms = v_search_ms(i-1);
        else
            v_curva_adherencia_ms = v_search_ms(1);
        end
        break;
    end
end

if v_curva_adherencia_ms == 0
    v_curva_adherencia_ms = max(v_search_ms);
end

v_curva_adherencia_kmh = v_curva_adherencia_ms * 3.6;

% --- 10.3 SELECCIÓN DE MARCHA Y VELOCIDAD MÁXIMA MECÁNICA DEL VEHÍCULO ---
optimal_gear_curva = 0;
v_max_gear_motriz_kmh = 0;

for g = 1:5  % 5ª marcha es la tope de velocidad real
    v_min_g = min(V_gear_kmh(g, :));
    v_max_g = max(V_gear_kmh(g, :));
    
    if v_curva_adherencia_kmh >= v_min_g && v_curva_adherencia_kmh <= v_max_g
        optimal_gear_curva = g;
        idx_pos = find(a_x_ms2(g, :) >= 0, 1, 'last');
        if ~isempty(idx_pos)
            v_max_gear_motriz_kmh = V_gear_kmh(g, idx_pos);
        else
            v_max_gear_motriz_kmh = v_max_g;
        end
        break;
    end
end

if optimal_gear_curva == 0
    optimal_gear_curva = 5;
    idx_pos = find(a_x_ms2(5, :) >= 0, 1, 'last');
    if ~isempty(idx_pos)
        v_max_gear_motriz_kmh = V_gear_kmh(5, idx_pos);
    else
        v_max_gear_motriz_kmh = max(V_gear_kmh(5, :));
    end
end

% --- 10.4 VELOCIDAD GOVERNANTE EN CURVA ---
v_curva_final_kmh = min(v_curva_adherencia_kmh, v_max_gear_motriz_kmh);
v_curva_final_ms  = v_curva_final_kmh / 3.6;

%% 11. MÓDULO PISTA (ÓVALO COMPLETO): SIMULACIÓN DE TRAMOS EN CADENA
% --- 11.1 GEOMETRÍA DEL ÓVALO Y PARÁMETROS DE PISTA ---
L_recta       = 400;                     % Longitud de cada recta [m]
L_curva       = pi * R_curva;            % Longitud de desarrollo del semicírculo [m] (~251.3 m)

% --- PARÁMETROS DE FRENADO CORREGIDOS ---
% Adherencia en frenado en los 4 neumáticos (90% del límite real mu)
mu_frenado    = 0.90 * car.mu;           % Coeficiente de fricción efectivo (~0.81 g)
F_brake_total = mu_frenado * (car.M * env.g * cos(env.theta)); % Fuerza total de frenado [N]
M_eq_frenado  = car.M * (1 + 0.05);      % Masa equivalente con inercia de frenado (~5%)

% Velocidad objetivo en curvas (m/s)
v_target_curva_ms = v_curva_final_ms;

% =========================================================================
% TRAMO 1: RECTA 1 (Aceleración desde 1000 RPM en 1ª marcha -> v_curva)
% =========================================================================
dt = 0.005;                              % Paso de integración temporal [s]
t_r1 = 0; 
d_r1 = 0; 

% Velocidad inicial a 1000 RPM en 1ª marcha (~8.20 km/h)
v_start_r1_ms = min(V_gear_kmh(1, :)) / 3.6;
v_r1 = v_start_r1_ms;

history_t_r1 = []; history_d_r1 = []; history_v_r1 = [];

while d_r1 < L_recta && t_r1 < 60
    % Selección de marcha activa
    curr_gear = 1;
    for g = 1:num_gears
        if v_r1 * 3.6 <= max(V_gear_kmh(g, :))
            curr_gear = g;
            break;
        end
    end
    
    % Interpolación directa sobre la curva de aceleración real (Sección 7)
    v_x_g = V_gear_kmh(curr_gear, :);
    a_y_g = a_x_real(curr_gear, :);
    [v_x_uniq, idx_u] = unique(v_x_g, 'stable');
    a_y_uniq = a_y_g(idx_u);
    
    a_inst = interp1(v_x_uniq, a_y_uniq, v_r1 * 3.6, 'linear', 'extrap');
    a_inst = max(0, a_inst);
    
    % Telemetría
    history_t_r1(end+1) = t_r1; 
    history_d_r1(end+1) = d_r1; 
    history_v_r1(end+1) = v_r1 * 3.6;
    
    % Integración Euler
    v_r1 = v_r1 + a_inst * dt;
    d_r1 = d_r1 + v_r1 * dt + 0.5 * a_inst * (dt^2);
    t_r1 = t_r1 + dt;
end

% --- Frenado Mínimo Corregido para Recta 1 ---
v_end_r1_ms = history_v_r1(end) / 3.6;
d_frenado_r1 = 0; t_frenado_r1 = 0;

if v_end_r1_ms > v_target_curva_ms
    v_brk = v_end_r1_ms:-0.05:v_target_curva_ms;
    for k = 1:length(v_brk)
        vk = v_brk(k); vk_mph = (vk * 3.6) / 1.60934;
        
        % Resistencias ambientales que ayudan a frenar
        F_res_k = 0.5*env.rho_air*car.A*car.Cd*(vk^2) + ...
                  (0.01*(1+(vk_mph/100)))*(car.M*env.g*cos(env.theta)) + ...
                  car.M*env.g*sin(env.theta);
              
        a_brk = (F_brake_total + F_res_k) / M_eq_frenado; % Desaceleración (~8 m/s^2)
        dt_k  = 0.05 / a_brk;
        
        d_frenado_r1 = d_frenado_r1 + vk * dt_k;
        t_frenado_r1 = t_frenado_r1 + dt_k;
    end
end

d_acc_efectiva_r1 = max(0, L_recta - d_frenado_r1);
idx_c1 = find(history_d_r1 >= d_acc_efectiva_r1, 1, 'first');
if isempty(idx_c1), idx_c1 = length(history_d_r1); end

t_acc_r1 = history_t_r1(idx_c1);
t_recta1_total = t_acc_r1 + t_frenado_r1;
v_max_recta1 = history_v_r1(idx_c1);

% =========================================================================
% TRAMO 2: CURVA 1 (Semicírculo 1 a Vel. Constante)
% =========================================================================
t_curva1_total = L_curva / v_target_curva_ms;

% =========================================================================
% TRAMO 3: RECTA 2 (Salida de Curva 1 v0 = v_curva -> v_curva)
% =========================================================================
t_r2 = 0; d_r2 = 0; 
v_r2 = v_target_curva_ms; % Arranca a la velocidad de salida de curva (60.60 km/h)

history_t_r2 = []; history_d_r2 = []; history_v_r2 = [];

while d_r2 < L_recta && t_r2 < 60
    curr_gear = 1;
    for g = 1:num_gears
        if v_r2 * 3.6 <= max(V_gear_kmh(g, :))
            curr_gear = g;
            break;
        end
    end
    
    v_x_g = V_gear_kmh(curr_gear, :);
    a_y_g = a_x_real(curr_gear, :);
    [v_x_uniq, idx_u] = unique(v_x_g, 'stable');
    a_y_uniq = a_y_g(idx_u);
    
    a_inst = interp1(v_x_uniq, a_y_uniq, v_r2 * 3.6, 'linear', 'extrap');
    a_inst = max(0, a_inst);
    
    history_t_r2(end+1) = t_r2; 
    history_d_r2(end+1) = d_r2; 
    history_v_r2(end+1) = v_r2 * 3.6;
    
    v_r2 = v_r2 + a_inst * dt;
    d_r2 = d_r2 + v_r2 * dt + 0.5 * a_inst * (dt^2);
    t_r2 = t_r2 + dt;
end

% --- Frenado Mínimo Corregido para Recta 2 ---
v_end_r2_ms = history_v_r2(end) / 3.6;
d_frenado_r2 = 0; t_frenado_r2 = 0;

if v_end_r2_ms > v_target_curva_ms
    v_brk = v_end_r2_ms:-0.05:v_target_curva_ms;
    for k = 1:length(v_brk)
        vk = v_brk(k); vk_mph = (vk * 3.6) / 1.60934;
        
        F_res_k = 0.5*env.rho_air*car.A*car.Cd*(vk^2) + ...
                  (0.01*(1+(vk_mph/100)))*(car.M*env.g*cos(env.theta)) + ...
                  car.M*env.g*sin(env.theta);
              
        a_brk = (F_brake_total + F_res_k) / M_eq_frenado;
        dt_k  = 0.05 / a_brk;
        
        d_frenado_r2 = d_frenado_r2 + vk * dt_k;
        t_frenado_r2 = t_frenado_r2 + dt_k;
    end
end

d_acc_efectiva_r2 = max(0, L_recta - d_frenado_r2);
idx_c2 = find(history_d_r2 >= d_acc_efectiva_r2, 1, 'first');
if isempty(idx_c2), idx_c2 = length(history_d_r2); end

t_acc_r2 = history_t_r2(idx_c2);
t_recta2_total = t_acc_r2 + t_frenado_r2;
v_max_recta2 = history_v_r2(idx_c2);

% =========================================================================
% TRAMO 4: CURVA 2 (Semicírculo 2 a Vel. Constante - Cierra la Vuelta)
% =========================================================================
t_curva2_total = L_curva / v_target_curva_ms;

% =========================================================================
% RESUMEN GENERAL Y TIEMPO TOTAL DE VUELTA
% =========================================================================
t_vuelta_completa = t_recta1_total + t_curva1_total + t_recta2_total + t_curva2_total;
distancia_lap = 2 * L_recta + 2 * L_curva;

%% 12. REPORTE FORMAL EN CONSOLA
fprintf('\n========================================================================\n');
fprintf('  TELEMETRÍA DE DESEMPEÑO - VUELTA COMPLETA AL ÓVALO (4 TRAMOS)\n');
fprintf('========================================================================\n');
fprintf('Distancia Total Trazado Óvalo:             %.1f m\n', distancia_lap);
fprintf('------------------------------------------------------------------------\n');
fprintf('TRAMO 1: RECTA 1 (L = %.0fm) - Arrancando a 1000 RPM en 1ª (%.2f km/h)\n', L_recta, v_start_r1_ms * 3.6);
fprintf('  - Vel. Inicial:                         %.2f km/h\n', v_start_r1_ms * 3.6);
fprintf('  - Vel. Máxima Alcanzada en Recta 1:     %.2f km/h\n', v_max_recta1);
fprintf('  - Distancia Acelerando a Fondo:         %.1f m (Tiempo: %.2f s)\n', d_acc_efectiva_r1, t_acc_r1);
fprintf('  - Distancia Mínima de Frenado (90%% Fx):   %.1f m (Tiempo: %.2f s)\n', d_frenado_r1, t_frenado_r1);
fprintf('  - Vel. Final de Entrada a Curva 1:      %.2f km/h\n', v_curva_final_kmh);
fprintf('  - Tiempo Total Recta 1:                 %.2f s\n', t_recta1_total);
fprintf('------------------------------------------------------------------------\n');
fprintf('TRAMO 2: CURVA 1 (R = %.0fm, L = %.1fm)\n', R_curva, L_curva);
fprintf('  - Velocidad Sostenida en Curva 1:       %.2f km/h\n', v_curva_final_kmh);
fprintf('  - Tiempo Total Curva 1:                 %.2f s\n', t_curva1_total);
fprintf('------------------------------------------------------------------------\n');
fprintf('TRAMO 3: RECTA 2 (L = %.0fm) - Salida de Curva 1 (v0 = %.2f km/h)\n', L_recta, v_curva_final_kmh);
fprintf('  - Vel. Inicial (Salida de Curva 1):     %.2f km/h\n', v_curva_final_kmh);
fprintf('  - Vel. Máxima Alcanzada en Recta 2:     %.2f km/h\n', v_max_recta2);
fprintf('  - Distancia Acelerando a Fondo:         %.1f m (Tiempo: %.2f s)\n', d_acc_efectiva_r2, t_acc_r2);
fprintf('  - Distancia Mínima de Frenado (90%% Fx):   %.1f m (Tiempo: %.2f s)\n', d_frenado_r2, t_frenado_r2);
fprintf('  - Vel. Final de Entrada a Curva 2:      %.2f km/h\n', v_curva_final_kmh);
fprintf('  - Tiempo Total Recta 2:                 %.2f s\n', t_recta2_total);
fprintf('------------------------------------------------------------------------\n');
fprintf('TRAMO 4: CURVA 2 (R = %.0fm, L = %.1fm) - Cierra el Circuito\n', R_curva, L_curva);
fprintf('  - Velocidad Sostenida en Curva 2:       %.2f km/h\n', v_curva_final_kmh);
fprintf('  - Tiempo Total Curva 2:                 %.2f s\n', t_curva2_total);
fprintf('------------------------------------------------------------------------\n');
fprintf('>>> TIEMPO TOTAL DE VUELTA AL ÓVALO:       %.2f SEGUNDOS <<<\n', t_vuelta_completa);
fprintf('========================================================================\n\n');