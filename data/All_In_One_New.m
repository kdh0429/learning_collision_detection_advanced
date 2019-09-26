clear all
clc
format long

%% For normalization

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
%% Training Set

Tool_list = ["0_00kg", "2_01kg", "5_01kg"];
Collision_Aggregate_Data = [];
Free_Aggregate_Data = [];

hz = 100;
num_data_type = 3; % i, q, qdot, q_desired, qdot_desired, dyna, q_abs, qdot_abs, ee_acc
num_input = 6*(num_data_type-1) + 1;
num_time_step = 5;

% 날짜별
FolderName = dir;
folder_idx = 1;
for time_step = 1:size(FolderName,1)
    if ((size(FolderName(time_step).name,2) > 5) && (strcmp(FolderName(time_step).name(1:6), 'robot1')))
        DataFolderList(folder_idx) = string(FolderName(time_step).name);
        folder_idx = folder_idx + 1;
    end
end

% 충돌
for joint_data = 1:size(DataFolderList,2)
    cd (DataFolderList(joint_data))
    FolderName = dir;
    for k = 1:size(FolderName,1)
        if strcmp(FolderName(k).name, 'collision')
            cd('collision')
            for tool_idx = 1:3
                cd (Tool_list(tool_idx));
                NumCollisionExpFolderName = dir;
                for collision_num =1:size(NumCollisionExpFolderName,1)-2
                    cd (int2str(collision_num))
                    Data = load('Reduced_DRCL_Data.txt');
                    Collision_Aggregate_Data = vertcat(Collision_Aggregate_Data, Data);
                    cd ..;
                end
                cd ..;
            end
            cd ..;
        end
    end
    cd ..;
end


CollisionProcessData= zeros(size(Collision_Aggregate_Data,1), num_input*num_time_step+2);
CollisionProcessDataIdx = 1;
recent_wrong_dt_idx = 0;

for k=num_time_step:size(Collision_Aggregate_Data,1)
    % Check time stamp
    dt_data = round(Collision_Aggregate_Data(k,1) - Collision_Aggregate_Data(k-1,1),3);
    if dt_data ~= 1/hz
        recent_wrong_dt_idx = k;
    end
    
    if k < recent_wrong_dt_idx + num_time_step
        continue
    end
    
    CollisionProcessData(CollisionProcessDataIdx,num_input*num_time_step+1) = Collision_Aggregate_Data(k,65);
    CollisionProcessData(CollisionProcessDataIdx,num_input*num_time_step+2) = 1-Collision_Aggregate_Data(k,65);
    for time_step=1:num_time_step
        for joint_data=1:6
            CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = 2*(Collision_Aggregate_Data(k-time_step+1,1+joint_data) - Collision_Aggregate_Data(k-time_step+1,31+joint_data) - MinCurrentDyna(joint_data)) / (MaxCurrentDyna(joint_data)-MinCurrentDyna(joint_data)) -1; % dyna_torque - current_torque
            CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = 2*(Collision_Aggregate_Data(k-time_step+1,25+joint_data) - MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data) - MinTrainingData(1,13+joint_data)) -1; % qdot
        end
        CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*6+time_step) = (norm(Collision_Aggregate_Data(k-time_step+1,62:64))-MinAcc) / (MaxAcc-MinAcc); % end effector acceleration
    end
    CollisionProcessDataIdx = CollisionProcessDataIdx +1;
end
disp(size(Collision_Aggregate_Data,1))
clear Collision_Aggregate_Data;



% 자유모션
for joint_data = 1:size(DataFolderList,2)
    cd (DataFolderList(joint_data))
    FolderName = dir;
    for k = 1:size(FolderName,1)
        if strcmp(FolderName(k).name, 'free')
            cd('free')
            for tool_idx = 1:3
                cd (Tool_list(tool_idx));
                NumFreeExpFolderName = dir;
                for collision_num =1:size(NumFreeExpFolderName,1)-2
                    cd (int2str(collision_num))
                    Data = load('Reduced_DRCL_Data.txt');
                    Free_Aggregate_Data = vertcat(Free_Aggregate_Data, Data);
                    cd ..;
                end
                cd ..;
            end
            cd ..;
        end
    end
    cd ..;
end


FreeProcessData= zeros(size(Free_Aggregate_Data,1), num_input*num_time_step+2);
FreeProcessDataIdx = 1;
recent_wrong_dt_idx = 0;

for k=num_time_step:size(Free_Aggregate_Data,1)
    % Check time stamp
    dt_data = round(Free_Aggregate_Data(k,1) - Free_Aggregate_Data(k-1,1),3);
    if dt_data ~= 1/hz
        recent_wrong_dt_idx = k;
    end
    
    if k < recent_wrong_dt_idx + num_time_step
        continue
    end
    
    FreeProcessData(FreeProcessDataIdx,num_input*num_time_step+1) = Free_Aggregate_Data(k,65);
    FreeProcessData(FreeProcessDataIdx,num_input*num_time_step+2) = 1-Free_Aggregate_Data(k,65);
    for time_step=1:num_time_step
        for joint_data=1:6
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = 2*(Free_Aggregate_Data(k-time_step+1,1+joint_data) - Free_Aggregate_Data(k-time_step+1,31+joint_data) - MinCurrentDyna(joint_data)) / (MaxCurrentDyna(joint_data)-MinCurrentDyna(joint_data)) -1; % dyna_torque - current_torque
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = 2*(Free_Aggregate_Data(k-time_step+1,25+joint_data) - MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data) - MinTrainingData(1,13+joint_data)) -1; % qdot
        end
        FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*6+time_step) = (norm(Free_Aggregate_Data(k-time_step+1,62:64))-MinAcc) / (MaxAcc-MinAcc); % end effector acceleration
    end
    FreeProcessDataIdx = FreeProcessDataIdx +1;
end
disp(size(Free_Aggregate_Data,1))
clear Free_Aggregate_Data;


DataAll = [CollisionProcessData(1:CollisionProcessDataIdx-1,:); FreeProcessData(1:FreeProcessDataIdx-1,:)];
clear CollisionProcessData FreeProcessData;
DataAllMix = DataAll(randperm(size(DataAll,1)),:);
clear DataAll;

csvwrite('TrainingData.csv', DataAllMix);

%% Validation Set

cd ValidationSet
Validation_Data = load('Reduced_DRCL_Data_Validation.txt');
cd ..

ValidationProcessData= zeros(size(Validation_Data,1), num_input*num_time_step+2);
ValidationProcessDataIdx = 1;
recent_wrong_dt_idx = 0;

for k=num_time_step:size(Validation_Data,1)
    % Check time stamp
    dt_data = round(Validation_Data(k,1) - Validation_Data(k-1,1),3);
    if dt_data ~= 1/hz
        recent_wrong_dt_idx = k;
    end
    
    if k < recent_wrong_dt_idx + num_time_step
        continue
    end
    
    ValidationProcessData(ValidationProcessDataIdx,num_input*num_time_step+1) = Validation_Data(k,65);
    ValidationProcessData(ValidationProcessDataIdx,num_input*num_time_step+2) = 1-Validation_Data(k,65);
    for time_step=1:num_time_step
        for joint_data=1:6
           ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = 2*(Validation_Data(k-time_step+1,1+joint_data) - Validation_Data(k-time_step+1,31+joint_data) - MinCurrentDyna(joint_data)) / (MaxCurrentDyna(joint_data)-MinCurrentDyna(joint_data)) -1; % dyna_torque - current_torque
           ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = 2*(Validation_Data(k-time_step+1,25+joint_data) - MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data) - MinTrainingData(1,13+joint_data)) -1; % qdot
        end
        ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*6+time_step) = (norm(Validation_Data(k-time_step+1,62:64))-MinAcc) / (MaxAcc-MinAcc); % end effector acceleration
    end
    ValidationProcessDataIdx = ValidationProcessDataIdx +1;
end
disp(size(Validation_Data,1))
clear Validation_Data;

csvwrite('ValidationData.csv', ValidationProcessData(1:ValidationProcessDataIdx-1,:));

%%
% Test set
cd TestSet
Testing_Data = load('Reduced_DRCL_Data_Test.txt');
cd ..

TestProcessData= zeros(size(Testing_Data,1), num_input*num_time_step+2);
TestProcessDataIdx = 1;
recent_wrong_dt_idx = 0;

for k=num_time_step:size(Testing_Data,1)
    % Check time stamp
    dt_data = round(Testing_Data(k,1) - Testing_Data(k-1,1),3);
    if dt_data ~= 1/hz
        recent_wrong_dt_idx = k;
    end
    
    if k < recent_wrong_dt_idx + num_time_step
        continue
    end
    
    TestProcessData(TestProcessDataIdx,num_input*num_time_step+1) = Testing_Data(k,65);
    TestProcessData(TestProcessDataIdx,num_input*num_time_step+2) = 1-Testing_Data(k,65);
    for time_step=1:num_time_step
        for joint_data=1:6
           TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = 2*(Testing_Data(k-time_step+1,1+joint_data) - Testing_Data(k-time_step+1,31+joint_data) - MinCurrentDyna(joint_data)) / (MaxCurrentDyna(joint_data)-MinCurrentDyna(joint_data)) -1; % dyna_torque - current_torque
           TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = 2*(Testing_Data(k-time_step+1,25+joint_data) - MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data) - MinTrainingData(1,13+joint_data)) -1; % qdot
        end
        TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*6+time_step) = (norm(Testing_Data(k-time_step+1,62:64))-MinAcc) / (MaxAcc-MinAcc); % end effector acceleration
    end
    TestProcessDataIdx = TestProcessDataIdx +1;
end
disp(size(Testing_Data,1))
clear Testing_Data;

csvwrite('TestingData.csv', TestProcessData(1:TestProcessDataIdx-1,:));

