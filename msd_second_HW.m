clear all;
close all;
clc;

% ** Capacitor Variables ** %
A = 0.0025;                          % Area of a single plate (m^2)
Cd = 46.48*10^-6;                    % Distance between plates (m)
Cdmin = 0.415*10^-6;                 % Minimum distance between plates (m)
m = 33.75*10^-3;                     % Weigth of a single plate (kg)
K = 2.1;                             % Dielectric constant of PTFE 
E0 = 8.85*10^-12;                    % Vacuum permittivity (C^2/Nm^2)

C = K*E0*A/Cd;

% fprintf('Capacitance = %d\n', C);

% ** Spring variables ** %
G = 7.72 * 10^10;                    % shear modulus of stainless steel 302 (N/m^2)
d = 1.2 * 10^-3;                     % diameter of the wire (m)
D = 14 * 10^-3;                      % mean diameter of the coils (m)
N = 10;                              % number of coils contributing to the spring's elasticity

k = G * d^4 /(8*D^3 * N); 

%fprintf('Spring constant k = %d\n', k);


% ** Circuit Variables ** %
Vin = 5;                             % Input Voltage (V)
R2 = 10^2;                           % R2 value (ohm)

% Function for R1 
a = 100;
b = 1;
R1 = @(x) a ./ (x + b);

% == Simulation Setup == %

odefun = @(t,y)[
    y(2); % y(1) = x, y(2) = v dx/dt = v
    (E0*A * Vin^2 * R1(y(1)) / ((R1(y(1)) + R2)^2 * 2 * Cd^2 * (y(1) + Cd) * m)) - (k / m) * y(1) % dv/dt
];

% Event function to limit x from going below -0.415×10^-6
% If x goes below -0.415×10^-6 the capacitance will short and won't work
function [value, isterminal, direction] = lowerLimitCondition(t, y)
    lower_limit = -0.415e-6; 
    value = y(1) - lower_limit;  
    isterminal = 0;  
    direction = -1;  
end

% == Initial conditions of the system == %
x0 = 0;
v0 = 0;
y0 = [x0; v0];

% time span 
tspan = [0,5];

% == Start the Simulation == %
options = odeset('Events', @lowerLimitCondition);
[t, y, te, ye, ie] = ode45(odefun, tspan, y0, options);

x = y(:, 1); %position
v = y(:, 2); %velocity

x(x <= -0.415e-6) = -0.415e-6;  % sets x to -0.415×10^-6 when it  reaches below


% == Plot the results == %
figure;
subplot(3, 1, 1);
plot(t, x, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (x)');
title('Position vs. Time');
grid on;

subplot(3, 1, 2);
plot(t, v, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (v)');
title('Velocity vs. Time');
grid on;

R1_values = R1(x);
subplot(3, 1, 3);
plot(x, R1_values, 'LineWidth', 1.5)
xlabel('X');
ylabel('R1 values');
title('X vs R1');
grid on;

% Display the final time of the system and thee final position
disp(['Final time: ', num2str(t(end)), ' seconds']);
disp(['Final position: ', num2str(x(end))]);

