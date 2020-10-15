% DATA FORMAT: [TIME X Y Z Q1 Q2 Q3 Q4 Q5 Q6]
data_PD = dlmread('PD_log.txt');
data_PD_desired = dlmread('PD_log_desired.txt');
data_LQR = dlmread('LQR_log.txt');
data_LQR_desired = dlmread('LQR_log_desired.txt');
data_QP = dlmread('QP_log.txt');
data_QP_desired = dlmread('QP_log_desired.txt');


% Plot Cartesian space graphs
% Period of trajectory = 2*pi
% Each joint moves by the law: q(i) = 0.2*cos(1.0*t)
T = 2*pi;
t_from = 5.0;
t_to = t_from + T;

t_idx_QP = (data_QP(:,1) >= t_from) & (data_QP(:,1) <= t_to);
t_idx_LQR = (data_LQR(:,1) >= t_from) & (data_LQR(:,1) <= t_to);
t_idx_PD = (data_PD(:,1) >= t_from) & (data_PD(:,1) <= t_to);


%% Cartesian space
% figure;
% title('XY plane');
% hold on;
% % plot desired trajectory
% plot(data_QP_desired(t_idx_QP,2), data_QP_desired(t_idx_QP,3));
% axis equal;
% 
% % plot actual trajectory
% plot(data_PD(t_idx_PD,2), data_PD(t_idx_PD,3));
% plot(data_LQR(t_idx_LQR,2), data_LQR(t_idx_LQR,3));
% plot(data_QP(t_idx_QP,2), data_QP(t_idx_QP,3));
% hold off;
% 
% 
% figure;
% title('XZ plane');
% hold on;
% % plot desired trajectory
% plot(data_QP_desired(t_idx_QP,2), data_QP_desired(t_idx_QP,4));
% axis equal;
% 
% % plot actual trajectory
% plot(data_PD(t_idx_PD,2), data_PD(t_idx_PD,4));
% plot(data_LQR(t_idx_LQR,2), data_LQR(t_idx_LQR,4));
% plot(data_QP(t_idx_QP,2), data_QP(t_idx_QP,4));
% hold off;


%% State space
q_index= 4; % initial index offset;
q_index = q_index + 3;
figure;
hold on;
% plot desired trajectory
plot(data_QP_desired(t_idx_QP,1), data_QP_desired(t_idx_QP,q_index));
% axis equal;

% plot actual trajectory
plot(data_PD(t_idx_PD,1), data_PD(t_idx_PD,q_index));
plot(data_LQR(t_idx_LQR,1), data_LQR(t_idx_LQR,q_index));
plot(data_QP(t_idx_QP,1), data_QP(t_idx_QP,q_index));
hold off;

