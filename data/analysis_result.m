clc
clear all
cd ..
cd RealRobotCode
Result = load("Result.txt");

l = Result(:,8:14) - Result(:,1:7);
logit = sum(Result(:,8:14) - Result(:,1:7),2)+Result(:,16)-Result(:,15);
res = 1./(1+exp(logit));


exp_modular_value = exp(Result(:,8:14) - Result(:,1:7));
for i=1:7
    subplot(2,4,i)
    plot(exp_modular_value(:,i))
    legend_name = strcat('M',int2str(i),'(','W2',int2str(i),'-','W1',int2str(i),')');
    legend(legend_name)
    hold off
end

subplot(2,4,8)
plot(l)
hold on
plot(Result(:,17))
legend('e^(M1(W21-W11))','e^(M2(W22-W12))','e^(M3(W23-W13))','e^(M4(W24-W14))','e^(M5(W25-W15))','e^(M6(W26-W16))','e^(M7(W27-W17))','NN Result')
hold off