clear all;

% DATA FORMAT: [TIME X Y Z]
data_QP_1e3 = dlmread('QP_log_01_100.txt');
data_QP_1e3_desired = dlmread('QP_log_01_100_desired.txt');
data_QP_2e3 = dlmread('QP_log_005_100.txt');
data_QP_2e3_desired = dlmread('QP_log_005_100_desired.txt');
data_QP_1e4 = dlmread('QP_log_001_100.txt');
data_QP_1e4_desired = dlmread('QP_log_001_100_desired.txt');
data_QP_1e5 = dlmread('QP_log_001_1000.txt');
data_QP_1e5_desired = dlmread('QP_log_001_1000_desired.txt');
data_QP_1e6 = dlmread('QP_log_0001_1000.txt');
data_QP_1e6_desired = dlmread('QP_log_0001_1000_desired.txt');
data_QP_1e7 = dlmread('QP_log_0001_10000.txt');
data_QP_1e7_desired = dlmread('QP_log_0001_10000_desired.txt');

% Plot Cartesian space graphs
% Period of trajectory = 10 s
T = 10;
t_from = 25.0;
t_to = t_from + T*1.2;

t_idx_QP_1e3 = (data_QP_1e3(:,1) >= t_from) & (data_QP_1e3(:,1) <= t_to);
t_idx_QP_2e3 = (data_QP_2e3(:,1) >= t_from) & (data_QP_2e3(:,1) <= t_to);
t_idx_QP_1e4 = (data_QP_1e4(:,1) >= t_from) & (data_QP_1e4(:,1) <= t_to);
t_idx_QP_1e5 = (data_QP_1e5(:,1) >= t_from) & (data_QP_1e5(:,1) <= t_to);
t_idx_QP_1e6 = (data_QP_1e6(:,1) >= t_from) & (data_QP_1e6(:,1) <= t_to);
t_idx_QP_1e7 = (data_QP_1e7(:,1) >= t_from) & (data_QP_1e7(:,1) <= t_to);

q_index_offset = 4; % data format offset
q_index = q_index_offset + 3;
figure;
xlabel('time, s');
ylabel(sprintf('q_%i, rad', q_index - q_index_offset));
hold on;
plot(data_QP_1e3_desired(t_idx_QP_1e3,1), data_QP_1e3_desired(t_idx_QP_1e3,q_index));
plot(data_QP_1e3(t_idx_QP_1e3,1), data_QP_1e3(t_idx_QP_1e3,q_index));
plot(data_QP_2e3(t_idx_QP_2e3,1), data_QP_2e3(t_idx_QP_2e3,q_index));
plot(data_QP_1e4(t_idx_QP_1e4,1), data_QP_1e4(t_idx_QP_1e4,q_index));
plot(data_QP_1e5(t_idx_QP_1e5,1), data_QP_1e5(t_idx_QP_1e5,q_index));
plot(data_QP_1e6(t_idx_QP_1e6,1), data_QP_1e6(t_idx_QP_1e6,q_index));
plot(data_QP_1e7(t_idx_QP_1e7,1), data_QP_1e7(t_idx_QP_1e7,q_index));

legend(...
    'desired', ...
    'cost_u = 0.1, cost_a = 100', ...
    'cost_u = 0.05, cost_a = 100', ...
    'cost_u = 0.01, cost_a = 100', ...
    'cost_u = 0.01, cost_a = 1000', ...
    'cost_u = 0.001, cost_a = 1000', ...
    'cost_u = 0.001, cost_a = 10000' ...
    );
    
