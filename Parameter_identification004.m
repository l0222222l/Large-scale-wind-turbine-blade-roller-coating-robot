%% 参数初始化
% IRLS与阻抗控制对比 刚度波动
clear; clc;

% 系统参数
m = 10;          % 质量 (kg)
b = 200;         % 阻尼系数 (Ns/m)
k = 50;          % 初始结构刚度 (N/m)
fd = 10;         % 期望接触力 (N)

time_step = 0.001;  
total_time = 3;     
t = 0:time_step:total_time;  

% 环境刚度
ke = zeros(size(t));
ke(t <= total_time) = (1 + 0.1 * sin(2 * pi * t / total_time*3)) * 10000; 
x_irls = 0;  v_irls = 0;  a_irls = 0;   
x_direct = 0; v_direct = 0; a_direct = 0; 

k_hat_irls = 100;   
P_irls = 100;       
lambda = 0.9;         
max_iter = 5;         

% 数据存储变量
fe_data_irls = zeros(size(t));        
fe_data_direct = zeros(size(t));      
k_hat_irls_data = zeros(size(t));     

%% 仿真循环
for i = 1:length(t)
    fe_irls = ke(i) * max(0, x_irls);          
    fe_direct = ke(i) * max(0, x_direct);      
    
    phi_irls = max(0, x_irls);             
    error_irls = fe_irls - k_hat_irls * phi_irls;  
    for iter = 1:max_iter
        weight = 1 / max(abs(error_irls), 1e-6);  
        K_gain_irls = P_irls * phi_irls * weight / (weight + phi_irls' * P_irls * phi_irls);
        k_hat_irls = k_hat_irls + K_gain_irls * error_irls;
        P_irls = (P_irls - K_gain_irls * phi_irls' * P_irls) / lambda;
        error_irls = fe_irls - k_hat_irls * phi_irls;  
    end

    Delta_x_irls = (fd - fe_irls) / k_hat_irls;  
    x_r_irls = x_irls + Delta_x_irls;           
    a_irls = (fd - fe_irls - b*v_irls - k*(x_irls - x_r_irls)) / m; 
    v_irls = v_irls + a_irls * time_step;     
    x_irls = x_irls + v_irls * time_step;      
    

    Delta_x_direct = (fd - fe_direct) / k;  
    x_r_direct = x_direct + Delta_x_direct;  
    a_direct = (fd - fe_direct - b*v_direct - k*(x_direct - x_r_direct)) / m;  
    v_direct = v_direct + a_direct * time_step;  
    x_direct = x_direct + v_direct * time_step;  
    
    fe_data_irls(i) = fe_irls;
    fe_data_direct(i) = fe_direct;
    k_hat_irls_data(i) = k_hat_irls;
end

%% 绘图对比
figure;

% 图1：接触力对比图 (单位为 N/mm)
subplot(2,1,1);
plot(t, fd * ones(size(t)), 'k', 'LineWidth', 1.5);  hold on; 
plot(t, fe_data_direct, 'r', 'LineWidth', 1.5);  
plot(t, fe_data_irls, 'b', 'LineWidth', 1.5);  
xlabel('时间 (s)');
ylabel('接触力 (N)');
legend('期望力','阻抗控制','IRLS');
grid on;

% 图2：刚度辨识对比图（单位为 N/mm）
subplot(2,1,2);
plot(t, ke / 1000, 'k', 'LineWidth', 1.5);  hold on; 
plot(t, k_hat_irls_data / 1000, 'b', 'LineWidth', 1.5);  
xlabel('时间 (s)');
ylabel('刚度 (N/mm)');
legend('实际环境刚度','IRLS刚度估计');
grid on;

