%% Please read Supplementary_Material.pdf first.

%% Description:

% This script shows the following results:
% [1] The pole that crosses the unit circle of z-plane for the first time
% is the one with the smallest theta value, and this pole uniquely
% determines the critical gain. This will be shown in Figure 1.

% [2] In general, alpha decreases with an increase in time 
% delay. This will be shown in Figure 2.

% [3] The longer the time delay, the more sensitive alpha
% becomes with respect to the vertical velocity Vz at different 
% linearization points near hover condition. This will be shown in Figure
% 3, and the maximum percentage deviation in alpha will be printed.

% Parameters:
% - m = mass
% - T = sampling period for descretization
% - rho_CD_A =  product of air density, drag coefficient, and reference
% area.
% - n = time delay as an integer number of T
% - Vz = Vertical velocity at the linearization point near hover condition

%%
clear all; close all; clc;

m = 0.5;
T = 0.01;
rho_CD_A = 1;
n_max = 30;
Vz_max = 0.5;
Vz_min = -0.5;

% There are n+1 zeros at equiangular infinities whose corresponding 
% asymptotes are 2*pi/(n+1) radians apart from each neighbor. 
% Since the Root Locus plot is vertical symmeteric, and this means we only
% need to consider ceil((n+1)/2) number of asymptotes to find the critical
% gain.
% Note that the first root locus trajectory crossing the unit circle will
% be approximately pi/(n+1) rad apart from positive real axis. Therefore,
% to find the exact theta, we evaluate a range of theta values between
% theta_min1 = pi/(2*(n+1)) 
% theta_max1 = theta_min1 + 2*pi/(n+1).
% For the second one, we evaluate theta between
% theta_min2 = theta_max2;
% theta_max2 = theta_min2 + 2*pi/(n+1)

results = [];
for n = 1:n_max
    asymptote_interval = 2*pi/(n+1);
    min_alpha = inf;
    max_alpha = 0;
    for Vz = linspace(Vz_min, Vz_max, 100)
        p = sign(Vz)*rho_CD_A/m*Vz;
        alpha_final = inf;
        for asymptote_idx = 1: ceil((n+1)/2)
            theta_min = asymptote_interval/4 + (asymptote_idx-1)*asymptote_interval;
            theta_max = theta_min + asymptote_interval;
            for theta = linspace(theta_min, theta_max, 5000)
                % Find theta by estimating the solution for
                %  exp(-p*T) = sin((n+1)*theta)/sin(n*theta)
                diff = exp(-p*T) - sin((n+1)*theta)/sin(n*theta);
                if (abs(diff) < 0.001)
                    alpha = m*p/(1-exp(-p*T))*sin(theta)/sin(n*theta);
                    if (alpha > 0 && alpha < alpha_final)
                        alpha_final = alpha;
                        idx_final = asymptote_idx;
                        if (alpha < min_alpha) 
                            min_alpha = alpha; 
                        end
                        if (alpha > max_alpha ) 
                            max_alpha = alpha;
                        end
                        break;
                    end
                end
            end
        end
    end
    midrange = (max_alpha + min_alpha)/2;
    deviation_percent = (max_alpha - midrange)/midrange*100;

    if (max_alpha ~= inf && min_alpha ~= 0)
        result = [n; idx_final; min_alpha; max_alpha; deviation_percent];
    end
    
    results = [results, result];

end

figure(1)
plot(results(1,:), results(2,:), 'o')
xlim([0 n])
ylim([0 5])
yticks([0 1 2 3 4 5])
ylabel('Asymptote index causing instability')
xlabel('n (= time delay as number of sampling periods)')

figure(2)
plot(results(1,:), results(3,:), 'o')
hold on;
plot(results(1,:), results(4,:), 'd')
xlim([0 n])
legend('Min', 'Max')
ylabel('alpha')
xlabel('n (= time delay as number of sampling periods)')

figure(3)
plot(results(1,:), results(5,:), 'o')
xlim([0 n])
ylabel('alpha Deviation [%]')
xlabel('n (= time delay as number of sampling periods)')

str = ['The maximum deviation of alpha caused by different values of Vz at different linearzation points is ', num2str(max(result(5,:))), '%'];
disp(str);


