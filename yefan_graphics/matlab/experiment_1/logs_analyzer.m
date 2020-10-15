% DATA FORMAT: [TIME X Y Z]
data_PD = dlmread('PD_log.txt');
data_PD_desired = dlmread('PD_log_desired.txt');
data_LQR = dlmread('LQR_log.txt');
data_LQR_desired = dlmread('LQR_log_desired.txt');
data_QP = dlmread('QP_log.txt');
data_QP_desired = dlmread('QP_log_desired.txt');


% Plot Cartesian space graphs
% Period of trajectory = 10s
T = 10;
t_from = 15.0;
t_to = t_from + T;

figure;
hold on;
t_idx_PD = (data_PD(:,1) >= t_from) & (data_PD(:,1) <= t_to);
% plot desired trajectory
plot(data_PD_desired(t_idx_PD,2), data_PD_desired(t_idx_PD,4));
axis equal;

% plot actual trajectory
plot(data_PD(t_idx_PD,2), data_PD(t_idx_PD,4));

t_idx_LQR = (data_LQR(:,1) >= t_from) & (data_LQR(:,1) <= t_to);
plot(data_LQR(t_idx_LQR,2), data_LQR(t_idx_LQR,4));

t_idx_QP = (data_QP(:,1) >= t_from) & (data_QP(:,1) <= t_to);
plot(data_QP(t_idx_QP,2), data_QP(t_idx_QP,4));

hold off;
