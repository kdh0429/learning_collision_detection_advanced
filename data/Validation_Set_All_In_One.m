clear all
clc
format long

Original_hz = 1000;
Reduced_hz = 100;
Reduced_interval = Original_hz / Reduced_hz;

DoosanMSeries_RoboticsToolBox_Simulator;
cd ValidationSet

%% Collision
ValidationData = load('DRCL_Data_Validation.txt');
ReducedDataValidation = zeros(fix(size(ValidationData,1)/Reduced_interval), size(ValidationData,2));
for row = 1:size(ReducedDataValidation,1)
    ReducedDataValidation(row,:) = ValidationData(Reduced_interval*(row-1)+1,:);
end

% 가속도 보상 & 동적토크 모터 관성 추가
AngleM = ReducedDataValidation(:,8:13);
VelD = ReducedDataValidation(:,26:31);
ToolAccM = ReducedDataValidation(:,62:64);
TorqueDyn = ReducedDataValidation(:,32:37);

ToolGrav = zeros(size(ReducedDataValidation,1),3);
ToolAccD = zeros(size(ReducedDataValidation,1),3);
BaseAccD = zeros(size(ReducedDataValidation,1),6);
AccD = zeros(size(ReducedDataValidation,1),6);
for m = 1:size(ReducedDataValidation,1)
    R_T = bot.fkine(AngleM(m,:)).R()';
    if m == 1
        AccD(m,:) = (VelD(m+1,:) - VelD(m,:))*Reduced_hz;
    else
        AccD(m,:) = (VelD(m,:) - VelD(m-1,:))*Reduced_hz;
    end
    ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
    BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*AccD(m,:)' + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
    if mod(m, 10000) == 0
        m
    end
end
for j = 1:6
    TorqueDyn(:,j) = TorqueDyn(:,j) + bot.links(j).Jm*bot.links(j).G*bot.links(j).G * AccD(:,j);
end
ReducedDataValidation(:,62:64) = ToolAccM+ToolGrav-ToolAccD;
ReducedDataValidation(:,32:37) = TorqueDyn;


%% Data Process
%% For normalization

process_hz = 100;
num_data_type = 6; % i-dyna, qdot, q_desired - q, qdot_desired - qdot, ee_acc
num_input = 6*(num_data_type-1) + 1;
num_time_step = 5;
num_collision_data_cut_start = 10;
num_collision_data_cut_finish = 15;
num_negative_edge_data_cut = 20;

MaxTrainingData = ...
[25, ... % 시간 
300,300,200,100,100,50, ... % 전류기반 토크 
3.14,3.14,1.57,3.14,3.14,3.14, ... % 엔코더 각도 
1.57,1.57,1.57,1.57,1.57,1.57, ... % 엔코더 각속도
3.14,3.14,1.57,3.14,3.14,3.14, ...  % 목표 각도
1.57,1.57,1.57,1.57,1.57,1.57, ... % 목표 각속도
300,300,200,100,100,50, ... % 동적 토크
3.14,3.14,1.57,3.14,3.14,3.14, ... % 절대엔코더 각도
1.57,1.57,1.57,1.57,1.57,1.57, ... % 절대엔코더 각속도 
-35.8125978300000,-61.6178708300000,-35.7297740300000,-15.0661117900000,-13.1961032600000,-12.8466518500000, ... % 추정 마찰력
60,60,60,60,60,60, ... % 온도
30,30,30, ... % 말단 가속도 
0,0,0, ... % 스위치, JTS충돌, 전류충돌 
300,300,200,100,100,50]; % JTS기반 토크 

MinTrainingData = ...
[25, ... % 시간 
-300,-300,-200,-100,-100,-50, ... % 전류기반 토크 
-3.14,-3.14,-1.57,-3.14,-3.14,-3.14, ... % 엔코더 각도 
-1.57,-1.57,-1.57,-1.57,-1.57,-1.57, ... % 엔코더 각속도
-3.14,-3.14,-1.57,-3.14,-3.14,-3.14, ... % 목표 각도
-1.57,-1.57,-1.57,-1.57,-1.57,-1.57, ... % 목표 각속도
-300,-300,-200,-100,-100,-50, ... % 동적 토크
-3.14,-3.14,-1.57,-3.14,-3.14,-3.14, ... % 절대엔코더 각도
-1.57,-1.57,-1.57,-1.57,-1.57,-1.57, ... % 절대엔코더 각속도 
-35.8125978300000,-61.6178708300000,-35.7297740300000,-15.0661117900000,-13.1961032600000,-12.8466518500000, ... % 추정 마찰력
5.0,5.0,5.0,5.0,5.0,5.0, ... % 온도
-30,-30,-30, ... % 말단 가속도 
0,0,0, ... % 스위치, JTS충돌, 전류충돌 
-300,-300,-200,-100,-100,-50]; % JTS기반 토크 


MaxCurrentDyna = 1.0e+02 *[1.446886991650000   1.668623672100000   0.975043730100000   0.366971076220000   0.388747072440000   0.155852918317100];
MinCurrentDyna = 1.0e+02 *[-1.707753304150000  -1.653056572400000  -0.922283913690000  -0.429723359370000  -0.222735385400000  -0.129340748762640];
MaxDeltaQdQ = [0.012106301100000   0.009996968500000   0.008732073000000   0.010951596900000   0.009698796749000   0.007999126000000];
MinDeltaQdQ = [-0.011341421000000  -0.007788007100000  -0.011375057900000  -0.011238997400000  -0.010040846200000  -0.009056478100000];
MaxDeltaQdotdQdot = [0.030471368000000   0.032628351300000   0.032703119000000   0.055313989900000   0.041539804000000   0.023768547000000];
MinDeltaQdotdQdot = [-0.037924034800000  -0.033170233000000  -0.037959729000000  -0.044833251500000  -0.044674244900000  -0.023149635200000];
MaxDeltaThetaQ = [0.001582920000000   0.002739420600000   0.002450675600000   0.003926920000000   0.000942399100000   0.001213046000000];
MinDeltaThetaQ = [-0.002181576000000  -0.003093831000000  -0.002274677500000  -0.001609872000000  -0.001274405000000  -0.001873778000000];
MaxAcc = 10;
MinAcc = 0;

%%
ValidationProcessData= zeros(size(ReducedDataValidation,1), num_input*num_time_step+2);
ValidationProcessDataIdx = 1;
recent_wrong_dt_idx = 0;
num_continuous_collision = 0;

for k=Reduced_hz/process_hz*num_time_step:size(ReducedDataValidation,1)
    % Check time stamp
    dt_data = round(ReducedDataValidation(k,1) - ReducedDataValidation(k-Reduced_hz/process_hz,1),3);
    if dt_data ~= 1/process_hz
        recent_wrong_dt_idx = k;
    end
    % Delete after negative edge
    if (ReducedDataValidation(k,65) == 0 && ReducedDataValidation(k-Reduced_hz/process_hz,65) ==1)
        recent_wrong_dt_idx = k+num_negative_edge_data_cut;
    end
    
    if k < recent_wrong_dt_idx + num_time_step
        continue
    end
    
    if (ReducedDataValidation(k,65) ==1)
        num_continuous_collision = num_continuous_collision + 1;
    else
        num_continuous_collision = 0;
    end
    
    if (((num_collision_data_cut_start< num_continuous_collision) && (num_continuous_collision < num_collision_data_cut_finish)) || num_continuous_collision == 0)
        ValidationProcessData(ValidationProcessDataIdx,num_input*num_time_step+1) = ReducedDataValidation(k,65);
        ValidationProcessData(ValidationProcessDataIdx,num_input*num_time_step+2) = 1-ReducedDataValidation(k,65);
        for time_step=1:num_time_step
            for joint_data=1:6
                ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = 2*(ReducedDataValidation(k-time_step+1,1+joint_data) - ReducedDataValidation(k-time_step+1,31+joint_data) - MinCurrentDyna(joint_data)) / (MaxCurrentDyna(joint_data)-MinCurrentDyna(joint_data)) -1; % dyna_torque - current_torque
                ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = 2*(ReducedDataValidation(k-time_step+1,13+joint_data) - MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data) - MinTrainingData(1,13+joint_data)) -1; % qdot
                ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+3) = 2*(ReducedDataValidation(k-time_step+1,19+joint_data) - ReducedDataValidation(k-time_step+1,7+joint_data) - MinDeltaQdQ(joint_data)) / (MaxDeltaQdQ(joint_data)-MinDeltaQdQ(joint_data)) -1; % dyna_torque - current_torque
                ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+4) = 2*(ReducedDataValidation(k-time_step+1,25+joint_data) - ReducedDataValidation(k-time_step+1,13+joint_data) - MinDeltaQdotdQdot(joint_data)) / (MaxDeltaQdotdQdot(joint_data)-MinDeltaQdotdQdot(joint_data)) -1; % dyna_torque - current_torque
                ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+5) = 2*(ReducedDataValidation(k-time_step+1,37+joint_data) - ReducedDataValidation(k-time_step+1,7+joint_data) - MinDeltaThetaQ(joint_data)) / (MaxDeltaThetaQ(joint_data)-MinDeltaThetaQ(joint_data)) -1; % qdot error
            end
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*6+time_step) = (norm(ReducedDataValidation(k-time_step+1,62:64))-MinAcc) / (MaxAcc-MinAcc); % end effector acceleration
        end
        ValidationProcessDataIdx = ValidationProcessDataIdx +1;
    end
end
disp(size(ReducedDataValidation,1))
clear ReducedDataValidation;

cd ..
csvwrite('ValidationData.csv', ValidationProcessData(1:ValidationProcessDataIdx-1,:));


