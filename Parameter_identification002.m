%% 参数初始化
% irls rls 刚度突变
clear; clc; close all;

% 系统参数
m = 10;          % 质量 (kg)
b = 200;         % 阻尼系数 (Ns/m)
k = 50;          % 初始结构刚度 (N/m)
fd = 10;         % 期望接触力 (N)

time_step = 0.001;  
total_time = 3;     
t = 0:time_step:total_time;  

% 环境刚度（阶跃变化）
ke = zeros(size(t));
ke(t <= total_time/2) = 5000;  
ke(t > total_time/2) = 10000;   
x  = 0;  v  = 0;  a  = 0;   
x_trad = 0;  v_trad = 0;  a_trad = 0;   

k_hat_irls = 1000;   
k_hat_rls  = 1000;   

P_irls = 100;
P_rls  = 100;
lambda = 0.9;         

% IRLS权重参数及迭代次数
max_iter = 5;

% 数据存储变量
fe_data = zeros(size(t));         
fe_trad_data = zeros(size(t));      
k_hat_irls_data = zeros(size(t));   
k_hat_rls_data = zeros(size(t));    

%% 仿真循环
for i = 1:length(t)
    fe = ke(i) * max(0, x);           
    fe_trad = ke(i) * max(0, x_trad);   
    
    phi = max(0, x);
    error_irls = fe - k_hat_irls * phi;
    
    for iter = 1:max_iter
        weight = 1 / max(abs(error_irls), 1e-6);  
        K_gain_irls = P_irls * phi * weight / (weight + phi' * P_irls * phi);
        k_hat_irls = k_hat_irls + K_gain_irls * error_irls;
        P_irls = (P_irls - K_gain_irls * phi' * P_irls) / lambda;
        error_irls = fe - k_hat_irls * phi;  
    end

    phi_rls = max(0, x_trad);
    K_gain_rls = P_rls * phi_rls / (1 + phi_rls' * P_rls * phi_rls);
    k_hat_rls = k_hat_rls + K_gain_rls * (fe_trad - phi_rls * k_hat_rls);
    P_rls = (P_rls - K_gain_rls * phi_rls' * P_rls) / 1;
    

    Delta_x_irls = (-fd  ) / k_hat_irls;
    x_r_irls = x + Delta_x_irls;

    a = (fd - fe - b*v - k*(x - x_r_irls)) / m;
    v = v + a * time_step;
    x = x + v * time_step;
    
    Delta_x_rls = (-fd ) / k_hat_rls;
    x_r_trad = x_trad + Delta_x_rls;
    a_trad = (fd - fe_trad - b*v_trad - k*(x_trad - x_r_trad)) / m;
    % 更新状态
    v_trad = v_trad + a_trad * time_step;
    x_trad = x_trad + v_trad * time_step;
    
    fe_data(i) = fe;
    fe_trad_data(i) = fe_trad;
    k_hat_irls_data(i) = k_hat_irls;
    k_hat_rls_data(i) = k_hat_rls;
end

%% 绘图对比
figure;

% 图1：接触力对比图 (单位为 N)
subplot(2,1,1);
plot(t, fd * ones(size(t)), 'k', 'LineWidth', 1.5); hold on; 
plot(t, fe_data, 'b', 'LineWidth', 1.5); hold on;
plot(t, fe_trad_data, 'r', 'LineWidth', 1.5);  
xlabel('时间 (s)');
ylabel('接触力 (N)'); 
ylim([0, 20]);
legend('期望力','IRLS', 'RLS');
grid on;

% 图2：刚度辨识对比图（单位为 N/mm）
subplot(2,1,2);
plot(t, k_hat_irls_data / 1000, 'b', 'LineWidth', 1.5); hold on;  
plot(t, k_hat_rls_data / 1000, 'r', 'LineWidth', 1.5);  
plot(t, ke / 1000, 'k', 'LineWidth', 1.5);  
xlabel('时间 (s)');
ylabel('刚度 (N/mm)'); 
ylim([0, 15]);
legend('IRLS估计', 'RLS估计', '真实环境刚度');
grid on;

