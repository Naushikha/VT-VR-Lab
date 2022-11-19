1; # https://stackoverflow.com/a/54013409

# Round to n decimal places implementation for Octave
# https://stackoverflow.com/a/24694128
function retval = round_n (n, d)
    retval = round(10^d * n) / 10^d;
endfunction

# wrapToPi implementation for Octave
# Clamps the angle into [-pi, pi]
# https://stackoverflow.com/a/27095188
function retval = wrapToPi(a)
    retval = a - 2 * pi * floor((a + pi) / (2 * pi));
endfunction

% DATA = [ time MMSI x y U chi ]
DATA = dlmread("../out/2022_11_19-17_48.octave.csv", ",");

% 4-state discrete-time XKF with motion prediction

A = unique(DATA(:, 2)); % find unique ship indexes in data set
idx = 1; % Ship#4, between Vanvikan - Trondheim#!# ONLY ONE SHIP IN DATASET
N1 = 110; % start sample
N2 = 150; % final sample

% extract all data for ship with index idx, store data in table ship1
j = 1;

for i = N1:N2

    if (DATA(i, 2) == A(idx))
        ship1(j, :) = DATA(i, :);
        j = j + 1;
    end

end

MMSI = ship1(1, 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion prediction data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h_p = 0.1; % 10 Hz plots

kmax = length(ship1);

k_p = 2; % round(kmax /2);% start sample k_p < k_max
tfinal = 30; % duration in seconds

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = round_n(ship1(:, 1), 2);
t = t - t(1); % make time relative to the first time entry
x = ship1(:, 3);
y = ship1(:, 4);
U = ship1(:, 5) * 0.514444; % knots to m/s

chi = atan2(diff([0; y]), diff([0; x])); % path-tangential angle
chi = wrapToPi(chi);

t_max = t(length(t)); % last time recorded (maximum)
t_sampling = mean(diff(t)); % mean time between messages

h = 0.025; % 40 Hz
k = 1;
M = round(t_max) / h;
N = length(t);
t_update = t(1);

% Initialization of kinematic observer: x = [x y U chi]
x_prd = x(1);
y_prd = y(1);
U_prd = U(1);
chi_prd = chi(1);
a = 0;
r = 0;

K1 = 10;
K2 = 10;
K3 = 30;
K4 = 50;

% Initialization of LTV Kalman filter
Q = diag([1 1 10 10]);
R = eye(4);

P = eye(4);
x_hat = [x(1) y(1) U(1) chi(1)]';

T_a = 10; % acceleration time constant
T_r = 50; % yaw rate time constant

B = [
    0 0
    0 0
    1 0
    0 1
    ];

H = eye(4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

simdata = zeros(M - 1, 11); % memory allocation

for i = 1:M - 1
    time = (i - 1) * h; % time (sec)

    % Store simulation data in a table
    simdata(i, :) = [time x_prd y_prd U_prd chi_prd x_hat' a r];

    % % Corrector with K = 0 (no update)
    % X_hat = X_prd;
    % P_hat = P_prd;

    % Measurements
    if (time >= t_update)
        x_k = x(k);
        y_k = y(k);
        U_k = U(k);
        chi_k = chi(k);

        % estimate of acceleration and yaw rate for sample k > 2
        if k > 2
            h1 = t(k) - t(k -1);
            h2 = t(k -1) - t(k -2);
            alp = ((h1 + h2) / h1)^2;

            if (h1 + h2) / 2 > 4% do not compute a and r if mean sampling time > 4s
                a_c = 0;
                r_c = 0;
            else
                a_c = ((1 - alp) * U(k) + alp * U(k -1) - U(k -2)) / ((1 - alp) * h1 + h2);
                r_c = ((1 - alp) * chi(k) + alp * chi(k -1) - chi(k -2)) / ((1 - alp) * h1 + h2);
            end

        else % zero for first two data points
            a_c = 0;
            r_c = 0;
        end

        % max values ( saturation ) to avoid estimates using wildpoints
        r_max = pi / 180;

        if r_c > r_max
            r_c = r_max;
        elseif r < -r_max
            r_c = -r_max;
        end

        % max values
        a_max = 1;

        if a_c > a_max
            a_c = a_max;
        elseif a_c < -a_max
            a_c = -a_max;
        end

        % Corrector Kalman tilter ( update states if new measurement )
        z_k = [x_k y_k U_k chi_k]';
        eps = z_k - H * x_hat;
        eps(4) = wrapToPi(eps(4));
        K = P * H' * inv(H * P * H' + R);
        x_hat = x_hat + K * eps;
        P = (eye(4) - K * H) * P * (eye(4) - K * H)' + K * R * K';

        % Corrector kinematic observer (update states if new measurement)
        x_prd = x_prd + h * K1 * (x_k - x_prd);
        y_prd - y_prd + h * K2 * (y_k - y_prd);
        U_prd = U_prd + h * K3 * (U_k - U_prd);
        chi_prd = chi_prd + h * K4 * wrapToPi (chi_k - chi_prd);

        if k < N
            k = k + 1;
            t_update = t(k);
        end

    end

    % Kalman filter model
    X_prd = [x_prd y_prd U_prd chi_prd]';
    f_prd = [
        X_prd(3) * cos(X_prd(4))
        X_prd(3) * sin(X_prd(4))
        0
        0
        ];

    F = [
        0 0 cos(X_prd(4)) -X_prd(3) * sin(X_prd(4))
        0 0 sin(X_prd(4)) X_prd(3) * cos(X_prd(4))
        0 0 0 0
        0 0 0 0
        ];

    PHI = eye(4) + h * F;

    % Predictor Kalman filter (k+1)
    x_hat = x_hat + h * (f_prd + F * (x_hat - X_prd) + B * [a r]');
    x_hat(4) = wrapToPi(x_hat(4));
    P = PHI * P * PHI' + Q;

    % Predictor kinematic observer (k+1)
    x_prd = x_prd + h * U_k * cos(chi_prd);
    y_prd = y_prd + h * U_k * sin(chi_prd);
    U_prd = U_prd + h * a;
    chi_prd = chi_prd + h * r;
    chi_prd = wrapToPi (chi_prd);
    a = a + (a_c -a) / T_a;
    r = r + (r_c -r) / T_r;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion prediction from time t(k_p)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_p(1) = x(k_p);
y_p(1) = y(k_p);
U_p(1) = U(k_p);
chi_p(1) = chi(k_p);

dt = t(k_p) - t(k_p -1);
a = (U(k_p) - U(k_p -1)) / dt;
r = wrapToPi (chi(k_p) - chi(k_p -1)) / dt;

for i = 1:tfinal / h_p
    x_p(i + 1) = x_p(i) + h_p * U_p(i) * cos(chi_p(i));
    y_p(i + 1) = y_p(i) + h_p * U_p(i) * sin(chi_p(i));
    U_p(i + 1) = U_p(i) + h_p * a;
    chi_p(i + 1) = chi_p(i) + h_p * r;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT SIMULATION DATA: x = [x y U chi] AND PREDICTED SHIP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_prd = simdata(:, 1);
x_prd = simdata(:, 2) / 1000;
y_prd = simdata(:, 3) / 1000;
U_prd = simdata(:, 4);
chi_prd = simdata(:, 5);

x_hat = simdata(:, 6) / 1000;
y_hat = simdata(:, 7) / 1000;
U_hat = simdata(:, 8);
chi_hat = simdata(:, 9);

a_hat = simdata(:, 10);
r_hat = simdata(:, 11);

x = x / 1000;
y = y / 1000;
x_p = x_p / 1000;
y_p = y_p / 1000;

figure(1)
xship = [-1, 1/3, 1/3, 1, 1/3, 1/3, -1]; % draw ship
yship = [-1/3, -1/3, -1/3, 0, 1/3, 1/3, 1/3];

g1 = hgtransform;
g2 = hgtransform;
patch('XData', xship, 'YData', yship, 'FaceColor', 'black', 'Parent', g1);
patch('XData', xship, 'YData', yship, 'FaceColor', 'black', 'Parent', g2);

hold on
plot(y, x, 'bo', y_prd, x_prd, 'r', 'LineWidth', 1);
plot(y_hat, x_hat, 'c', 'LineWidth', 1);
%plot(y(k_p),x(k_p),'rp ','MarkerSize ' ,30);
hold off; grid

set(gca, 'fontsize', 12)
xlabel('y [km]'); ylabel('x [km]')
title('xy-plot: Kinematic observer (red) and XKF (cyan)')

hold on
quiver(y_p(1), x_p(1), y_p(tfinal / h_p) - y_p(1), x_p(tfinal / h_p) - x_p(1), 1, 'g', 'LineWidth', 5)
% g1.Matrix = makehgtform('translate', [y_p(1),x_p(1),0], 'scale',0.6, 'zrotate',pi/2-chi_p(1));
% g2.Matrix = makehgtform('translate', [y_p(tfinal/h_p),x_p(tfinal/h_p),0], 'scale',0.6, 'zrotate',pi/2-chi_p(tfinal/h_p));
drawnow
hold off

figure(2)
subplot(611)
plot(t, (180 / pi) * chi, 'o', t_prd, (180 / pi) * chi_prd, 'r', t_prd, (180 / pi) * chi_hat, 'c'), grid, title('\chi [deg]');
set(gca, 'fontsize', 12)
subplot(612)
plot(t, U, 'o', t_prd, U_prd, 'r', t_prd, U_hat, 'c'), grid, title('U [m/s]');
subplot(613)
plot(t, x, 'o', t_prd, x_prd, 'r', t_prd, x_hat, 'c'), grid, title('x [km]');
set(gca, 'fontsize', 12)
subplot(614)
plot(t, y, 'o', t_prd, y_prd, 'r', t_prd, y_hat, 'c'), grid, title('y [km]');
set(gca, 'fontsize', 12)
subplot(615)
plot(t_prd, a_hat, 'b', 'LineWidth', 1), grid, title('a [m/s^2]');
set(gca, 'fontsize', 12)
subplot(616)
plot(t_prd, r_hat * 180 / pi, 'b', 'LineWidth', 1), grid, title('r [deg/s]');
set(gca, 'fontsize', 12)

waitfor(gcf)
