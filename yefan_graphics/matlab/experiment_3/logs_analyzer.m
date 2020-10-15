% DATA FORMAT: [TIME X Y Z]
data_PD = dlmread('PD_log.txt');
data_PD_desired = dlmread('PD_log_desired.txt');
data_LQR = dlmread('LQR_log.txt');
data_LQR_desired = dlmread('LQR_log_desired.txt');
data_QP = dlmread('QP_log.txt');
data_QP_desired = dlmread('QP_log_desired.txt');

R_PD = reshape(data_PD(:, [2:4, 6:8, 10:12])', [3 3 length(data_PD)]);
R_PD_desired = reshape(data_PD_desired(:, [2:4, 6:8, 10:12])', [3 3 length(data_PD_desired)]);
R_LQR = reshape(data_LQR(:, [2:4, 6:8, 10:12])', [3 3 length(data_LQR)]);
R_LQR_desired = reshape(data_LQR_desired(:, [2:4, 6:8, 10:12])', [3 3 length(data_LQR_desired)]);
R_QP = reshape(data_QP(:, [2:4, 6:8, 10:12])', [3 3 length(data_QP)]);
R_QP_desired = reshape(data_QP_desired(:, [2:4, 6:8, 10:12])', [3 3 length(data_QP_desired)]);


% Plot Cartesian space graphs
% Period of trajectory = 10 s
T = 10;
t_from = 15.0;
t_to = t_from + T;

t_idx_QP = (data_QP(:,1) >= t_from) & (data_QP(:,1) <= t_to);
t_idx_LQR = (data_LQR(:,1) >= t_from) & (data_LQR(:,1) <= t_to);
t_idx_PD = (data_PD(:,1) >= t_from) & (data_PD(:,1) <= t_to);


