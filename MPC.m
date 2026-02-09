%% 平面1m
clc;
clear;
close all;
%% 参数初始化
m = 5; % 质量 (kg)
b = 30; % 阻尼系数 (Ns/m)
k = 10; % 刚度 (N/m)

time_step = 0.001; 
total_time = 8; 
t = 0:time_step:total_time; 

ke = zeros(size(t));
ke(t <= total_time) = 1500; 
xe = 1; 
fd = 10; 

% 预测控制相关参数
N = 5; 
Nu = 1; 
lambda = 10; 

% 初始化系统变量
x = xe; 
v = 0; 
a = 0; 
fe = 0; 

% 初始化阻抗控制变量
x_impedance = xe; 
v_impedance = 0; 
a_impedance = 0; 
fe_impedance = 0; 
m_impedance = m;
b_impedance = b;
k_impedance = k;

% 数据存储
x_data = zeros(size(t));
fe_data = zeros(size(t));
m_data = zeros(size(t));
b_data = zeros(size(t));
k_data = zeros(size(t));
x_impedance_data = zeros(size(t));
fe_impedance_data = zeros(size(t));

%% 仿真循环
for i = 1 :length(t)
    fe = ke(i) * max(0, x - xe); 

    A = zeros(N, 3 * Nu); 
    E = ones(N, 1); 

    x_future = x;
    v_future = v;
    a_future = a;

    fe_future = zeros(N, 1);

    for j = 1:N
        if j > 1
            a_future = (fd - fe_future(j-1) - b * v_future - k * (x_future - xe)) / m; 
            v_future = v_future + a_future * time_step;
            x_future = x_future + v_future * time_step;
        end

        fe_future(j) = ke(i) * max(0, x_future - xe);

        phi_future = [-a_future, -v_future, -(x_future - xe)]; 
        for k_idx = 1:Nu
            if j >= k_idx
                col_start = (k_idx - 1) * 3 + 1; 
                col_end = col_start + 2;         
                A(j, col_start:col_end) = phi_future; 
            end
        end
    end

    y_k = fd - fe_future(1);

    H = A' * A + lambda * eye(3 * Nu);
    f = -A' * (E .* (fd - fe));
    deltaU = quadprog(H, f);

    g = [0.03; 0.03; 0.03]; 
    m = m + g(1) * deltaU(1); 
    b = b + g(2) * deltaU(2); 
    k = k + g(3) * deltaU(3);

    a = (fd - fe - b * v - k * (x - xe)) / m;
    v = v + a * time_step;
    x = x + v * time_step;

    fe_impedance = ke(i) * max(0, x_impedance - xe); 
    a_impedance = (fd - fe_impedance - b_impedance * v_impedance - k_impedance * (x_impedance - xe)) / m_impedance; 
    v_impedance = v_impedance + a_impedance * time_step; 
    x_impedance = x_impedance + v_impedance * time_step; 

    x_data(i) = x;
    fe_data(i) = fe;
    m_data(i) = m;
    b_data(i) = b;
    k_data(i) = k;
    x_impedance_data(i) = x_impedance;
    fe_impedance_data(i) = fe_impedance;
end
x_data = xe - abs(xe - x_data); 
x_impedance_data = xe - abs(xe - x_impedance_data);

%% 结果可视化
figure;
subplot(3, 1, 1);
plot(t, fd * ones(size(t)), 'b', 'LineWidth', 1.5);
hold on;
plot(t, fe_impedance_data, 'g', 'LineWidth', 1.5);
hold on;
plot(t, fe_data, 'r', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('接触力 (N)');
legend('期望力', '阻抗控制','离散时间MPC');
grid on;

subplot(3, 1, 2);
plot(t, x_data, 'LineWidth', 1.5);
hold on;
plot(t, x_impedance_data, 'g', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置 (m)');
ylim([0.97,1.02]);
legend('离散时间MPC', '阻抗控制');
grid on;

subplot(3, 1, 3);
plot(t, m_data, 'r', 'LineWidth', 1.5);
hold on;
plot(t, b_data, 'g', 'LineWidth', 1.5);
plot(t, k_data, 'b', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('参数值');
legend('惯性参数 m', '阻尼系数 b', '刚度参数 k');
grid on;
