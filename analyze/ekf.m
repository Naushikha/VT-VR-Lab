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

% 4-state discrete-time EKF with motion prediction (a = r = 0)

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
% Measurements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = round_n(ship1(:, 1), 2);
t = t - t(1); % makes time relative to the first time entry
x = ship1(:, 3);
y = ship1(:, 4);
U = ship1(:, 5) * 0.514444; % knots to m/s
chi = atan2(diff([0; y]), diff([0; x])); % path-tangential angle
chi = wrapToPi(chi);

t_max = t(length(t)); % last time recorded (maximum)
t_sampling = mean(diff(t)); % mean time between messages

h = 0.02; % 50 Hz
k = 1;
M = round(t_max) / h;
N = length(t);
t_update = t(1);

% initialization of EKF: X = [x y U chi]
Q = diag([0.01 0.01 0.1 0.1]);
R = diag([0.001 0.001 0.001 0.01]);
X_prd = [x(1) y(1) U(1) chi(1)]';
P_prd = 0.1 * eye(4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

simdata = zeros(M - 1, 7); % memory allocation

for i = 1:M - 1
    time = (i - 1) * h; % time (sec)

    % Corrector with K = 0 (no update)
    X_hat = X_prd;
    P_hat = P_prd;

    % Measurements
    if (time >= t_update)
        x_k = x(k);
        y_k = y(k);
        U_k = U(k);
        chi_k = chi(k);

        z_k = [x_k y_k U_k chi_k]';
        eps = z_k - X_prd;
        eps(4) = wrapToPi(eps(4));

        % Corrector
        K = P_prd * inv(P_prd + R);

        X_hat = X_prd + K * eps;
        P_hat = (eye(4) - K) * P_prd * (eye(4) - K)' + K * R * K';

        % Make sure not to exceed the number of samples available
        if k < N
            k = k + 1;
            t_update = t(k);
        end

    end

    % Store simulation data in a table
    simdata(i, :) = [ time X_prd' P_prd(1, 1) P_prd(2, 2) ];

    % Predictor (k+1)
    f_hat = [
        X_hat(3) * cos(X_hat(4))
        X_hat(3) * sin(X_hat(4))
        0
        0
    ]; % column vector

    A = [
        0 0 cos(X_hat(4)) -X_hat(3) * sin(X_hat(4))
        0 0 sin(X_hat(4)) X_hat(3) * cos(X_hat(4))
        0 0 0 0
        0 0 0 0
        ];

    PHI = eye(4) + A * h;
    X_prd = X_hat + h * f_hat;
    P_prd = PHI * P_hat * PHI' + Q;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT SIMULATION DATA: x = [x y U chi] AND PREDICTED SHIP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_prd = simdata(:, 1);
x_prd = simdata(:, 2) / 1000;
y_prd = simdata(:, 3) / 1000;
U_prd = simdata(:, 4);
chi_prd = simdata(:, 5);
chi_prd = wrapToPi(chi_prd);

P11 = simdata(:, 6);
P22 = simdata(:, 7);

x = x / 1000;
y = y / 1000;

figure(1)

hold on;
plot(y, x, 'bo', y_prd, x_prd, 'r', 'LineWidth', 1);
hold off;
grid on;

set(gca, 'fontsize', 12)
xlabel('y [km]'); ylabel('x [km]')
title('xy-plot: EKF (red) and AIS measurements (blue)')

figure(2)
subplot(411)
plot(t, (180 / pi) * chi, 'o', t_prd, (180 / pi) * chi_prd, 'r'), grid, title('\chi [deg]');
set (gca, 'fontsize', 12)
subplot(412)
plot(t, U, 'o', t_prd, U_prd, 'r', 'LineWidth', 1), grid, title('U [m/s]');
subplot(413)
plot(t, x, 'o', t_prd, x_prd, 'r', 'LineWidth', 1), grid, title('x [km]');
set (gca, 'fontsize', 12)
subplot(414)
plot(t, y, 'o', t_prd, y_prd, 'r', 'LineWidth', 1), grid, title('y [km]');
set (gca, 'fontsize', 12)

waitfor(gcf)# Prevents the script from closing
