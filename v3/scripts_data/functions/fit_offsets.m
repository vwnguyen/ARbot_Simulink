% Fit offsets
% Date: 05/20

distances = [0.1002 0.2256 0.4020 0.5453 0.6269 0.9029 1.2291 1.6046]

spds = [0.5000    0.7500    1.0000    1.1295    1.2500    1.5000    1.7500    2.0000 ]

acc_times = [0.4333    0.6333    0.8333    0.9667    1.0333    1.2333    1.4333    1.6333]

c = polyfit(spds,acc_times,1) % t1 time for the acceleration to reach 0
acc_model_lin = polyfit(spds,distances,1)
acc_model_quad = polyfit(spds,distances,2) % d1 distance it traveled in time t1
acc_model_cubic = polyfit(spds,distances,3)

%polyval(fit model (coefficients) , speed)

plot(spds,distances)
hold on
plot(spds,polyval(acc_model_lin,spds))
plot(spds,polyval(acc_model_quad,spds))
plot(spds,polyval(acc_model_cubic,spds))
legend('reg', 'lin', 'quad', 'cubic')
calc_diff = [distances' polyval(acc_model_lin,spds)' polyval(acc_model_quad,spds)' polyval(acc_model_cubic,spds)']

for i = 1:length(calc_diff)
    for j = 2:4
        calc_diff(i,j+3) = calc_diff(i,1) - calc_diff(i,j);
    end
end

calc_diff
    