%% 阻抗参数影响
clc;
clear;
close all;

%% 参数初始化
m_values = [1, 5, 10, 20];    % 不同的质量值 (kg)
b_values = [50, 100, 150, 200]; % 不同的阻尼系数 (Ns/m)
k_values = [10, 100, 200, 300];    % 不同的系统刚度 (N/m)

time_step = 0.001;  
total_time = 5;    
t = 0:time_step:total_time; 

ke = zeros(size(t));
ke(t <= total_time) = 1500; 
fd = 10; 

%% 仿真和绘图
figure;

% 图1：不同质量对系统响应的影响
subplot(3, 1, 1);
hold on;
for m = m_values
    [x_data, fe_data] = simulate_system(m, 150, 10, t, ke, fd, time_step);
    plot(t, fe_data, 'LineWidth', 1.5, 'DisplayName', ['m = ', num2str(m)]);
end
xlabel('时间 (s)');
ylabel('接触力 (N)');
legend show;
grid on;

% 图2：不同阻尼系数对系统响应的影响
subplot(3, 1, 2);
hold on;
for b = b_values
    [x_data, fe_data] = simulate_system(5, b, 10, t, ke, fd, time_step);
    plot(t, fe_data, 'LineWidth', 1.5, 'DisplayName', ['b = ', num2str(b)]);
end
xlabel('时间 (s)');
ylabel('接触力 (N)');
legend show;
grid on;

% 图3：不同系统刚度对系统响应的影响
subplot(3, 1, 3);
hold on;
for k = k_values
    [x_data, fe_data] = simulate_system(5, 150, k, t, ke, fd, time_step);
    plot(t, fe_data, 'LineWidth', 1.5, 'DisplayName', ['k = ', num2str(k)]);
end
xlabel('时间 (s)');
ylabel('接触力 (N)');
legend show;
grid on;

%% 仿真函数
function [x_data, fe_data] = simulate_system(m, b, k, t, ke, fd, time_step)
    x = 0; 
    v = 0; 
    a = 0; 
    fe = 0; 
    
    x_data = zeros(size(t));
    fe_data = zeros(size(t));
    
    for i = 1:length(t)
        fe = ke(i) * x; 
        
        a = (fd - fe - b * v - k * x) / m; 
        v = v + a * time_step;
        x = x + v * time_step;
        
        x_data(i) = x;
        fe_data(i) = fe;
    end
end