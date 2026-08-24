%% EV Charging & Power Electronics Simulation
% Averaged DC-DC buck converter model for EV battery charging.
% Standalone MATLAB simulation.

clear;
clc;
close all;

%% Parameters

Vin = 500;                 % DC input voltage [V]
Vbat_start = 360;          % Initial battery voltage [V]
Vbat_end = 400;            % Final battery voltage [V]

L = 2e-3;                  % Inductor [H]
C = 2.2e-3;                % Output capacitor [F]

fs = 20e3;                 % Switching frequency [Hz]
dt = 1e-6;                 % Simulation time step [s]
T = 0.50;                  % Simulation duration [s]

Iref = 18;                 % Charging current reference [A]

Rinductor = 0.08;          % Inductor resistance [ohm]
Rswitch = 0.04;            % Equivalent switch resistance [ohm]
Paux = 20;                 % Auxiliary power [W]

%% Time vector

t = 0:dt:T;
N = numel(t);

%% Pre-allocation

iL = zeros(1,N);
Vout = zeros(1,N);

Pin = zeros(1,N);
Pout = zeros(1,N);
eta = zeros(1,N);

Vbat = zeros(1,N);
Dcmd = zeros(1,N);

Vout(1) = Vbat_start;

%% DC-DC Converter Simulation

for k = 1:N-1

    % Battery voltage increases during charging
    Vbat(k) = Vbat_start + ...
        (Vbat_end - Vbat_start) * t(k)/T;

    % Feed-forward duty ratio
    duty_ff = Vbat(k) / Vin;

    % Voltage error
    voltage_error = Vbat(k) - Vout(k);

    % Duty ratio control
    duty = duty_ff + 0.0025 * voltage_error;

    % Duty ratio limits
    duty = min(max(duty,0.05),0.95);

    Dcmd(k) = duty;

    % Inductor voltage
    vL = duty*Vin - Vout(k) - iL(k)*Rinductor;

    % Inductor current
    iL(k+1) = max(0, ...
        iL(k) + (vL/L)*dt);

    % Capacitor current
    iC = iL(k) - Vout(k)/20;

    % Output voltage
    Vout(k+1) = max(0, ...
        Vout(k) + (iC/C)*dt);

    % Output power
    Pout(k) = Vout(k) * iL(k);

    % Estimated losses
    Ploss = iL(k)^2 * ...
        (Rinductor + Rswitch) + Paux;

    % Input power
    Pin(k) = Pout(k) + Ploss;

    % Efficiency
    eta(k) = 100 * Pout(k) / ...
        max(Pin(k),eps);

end

%% Final values

Vbat(end) = Vbat_end;

Dcmd(end) = Dcmd(end-1);
Pin(end) = Pin(end-1);
Pout(end) = Pout(end-1);
eta(end) = eta(end-1);

%% Display Results

fprintf('\n');
fprintf('========================================\n');
fprintf(' EV CHARGING POWER ELECTRONICS MODEL\n');
fprintf('========================================\n');

fprintf('Input Voltage          : %.1f V\n',Vin);

fprintf('Initial Battery Voltage: %.1f V\n', ...
    Vbat_start);

fprintf('Final Battery Voltage  : %.1f V\n', ...
    Vbat_end);

fprintf('Peak Output Power      : %.1f W\n', ...
    max(Pout));

fprintf('Average Efficiency     : %.2f %%\n', ...
    mean(eta(t > 0.1)));

fprintf('Final Output Voltage   : %.1f V\n', ...
    Vout(end));

fprintf('========================================\n');

%% Create Results Folder

if ~exist('results','dir')
    mkdir('results');
end

%% Result 1 - Output Voltage

figure;

plot(t,Vout,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Output Voltage (V)');

title('EV Charger Output Voltage');

saveas(gcf, ...
    'results/output-voltage.png');

%% Result 2 - Charging Current

figure;

plot(t,iL,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Charging Current (A)');

title('EV Charger Charging Current');

saveas(gcf, ...
    'results/charging-current.png');

%% Result 3 - Power Flow

figure;

plot(t,Pin/1000,'LineWidth',1.5);

hold on;

plot(t,Pout/1000,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Power (kW)');

title('Input and Output Power');

legend('Input Power', ...
       'Output Power', ...
       'Location','best');

saveas(gcf, ...
    'results/power-flow.png');

%% Result 4 - Efficiency

figure;

plot(t,eta,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Efficiency (%)');

title('DC-DC Converter Efficiency');

ylim([0 105]);

saveas(gcf, ...
    'results/efficiency.png');

%% Save Numerical Results

results.time = t;

results.input_voltage = Vin;

results.battery_voltage = Vbat;

results.output_voltage = Vout;

results.charging_current = iL;

results.input_power = Pin;

results.output_power = Pout;

results.efficiency = eta;

results.duty_ratio = Dcmd;

save('results/simulation_results.mat', ...
     'results');

fprintf('\nSimulation results saved successfully.\n');
