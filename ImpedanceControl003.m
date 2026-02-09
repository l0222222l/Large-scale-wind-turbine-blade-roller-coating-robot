%% 不同环境刚度下的接触力曲线
clc;
clear;
close all;

%% 参数初始化
m = 10; % 质量 (kg)
b = 200; % 阻尼系数 (Ns/m)
k = 0; % 初始结构刚度 (N/m)

time_step = 0.001; 
total_time = 5; 
t = 0:time_step:total_time; 

ke_values = [1, 5, 10, 20];  
ke_values = ke_values * 1000; 
be = 50;      
fd = 10; 

%% 初始化变量
v_initial = 0;
colors = lines(length(ke_values)); 
fe_semi_data = zeros(size(t));

% 绘图
figure;
hold on;
xlabel('时间 (s)');
ylabel('接触力 (N)');
grid on;

for j = 1:length(ke_values)
    x_semi = 0; 
    v_semi = v_initial; 
    a_semi = 0; 
    x_r = 0;    
    ke = ke_values(j); 

    fe_semi_data = zeros(size(t)); 

    for i = 1:length(t)
        if x_semi > x_r
            fe_semi = ke * (x_semi - x_r) + be * v_semi; % Kelvin-Voigt模型
        else
            fe_semi = 0; 
        end

        a_semi = (fd - fe_semi - b * v_semi - k * x_semi) / m; 
        v_semi = v_semi + a_semi * time_step; 
        x_semi = x_semi + v_semi * time_step; 

        fe_semi_data(i) = fe_semi;
    end

    plot(t, fe_semi_data, 'LineWidth', 1.5, 'DisplayName', ['环境刚度 = ' num2str(ke_values(j)/1000) ' N/mm'], 'Color', colors(j, :));
end

plot(t, fd * ones(size(t)), '--k', 'LineWidth', 1.5, 'DisplayName', ['期望力 = ' num2str(fd) ' N']);

legend;
