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

%% Training Set

Tool_list = ["0_00kg", "2_01kg", "5_01kg"];
Collision_Aggregate_Data = [];
Free_Aggregate_Data = [];

hz = 100;
num_data_type = 10; % i, q, qdot, q_desired, qdot_desired, dyna, q_abs, qdot_abs, temperature, ee_acc
num_input = 6*(num_data_type-1) + 3 * 1;
num_time_step = 5;

% 날짜별
FolderName = dir;
folder_idx = 1;
for time_step = 1:size(FolderName,1)
    if strcmp(FolderName(time_step).name(1), '2')
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
    
    if (Collision_Aggregate_Data(k,65) == 1)
        CollisionProcessData(CollisionProcessDataIdx,num_input*num_time_step+1) = Collision_Aggregate_Data(k,65);
        CollisionProcessData(CollisionProcessDataIdx,num_input*num_time_step+2) = 1-Collision_Aggregate_Data(k,65);
        for time_step=1:num_time_step
            for joint_data=1:6
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = (Collision_Aggregate_Data(k-time_step+1,1+joint_data)-MinTrainingData(1,1+joint_data)) / (MaxTrainingData(1,1+joint_data)-MinTrainingData(1,1+joint_data)); % current
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = (Collision_Aggregate_Data(k-time_step+1,7+joint_data)-MinTrainingData(1,7+joint_data)) / (MaxTrainingData(1,7+joint_data)-MinTrainingData(1,7+joint_data)); % q
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+3) = (Collision_Aggregate_Data(k-time_step+1,13+joint_data)-MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data)-MinTrainingData(1,13+joint_data)); % qdot
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+4) = (Collision_Aggregate_Data(k-time_step+1,19+joint_data)-MinTrainingData(1,19+joint_data)) / (MaxTrainingData(1,19+joint_data)-MinTrainingData(1,19+joint_data)); % q_desired
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+5) = (Collision_Aggregate_Data(k-time_step+1,25+joint_data)-MinTrainingData(1,25+joint_data)) / (MaxTrainingData(1,25+joint_data)-MinTrainingData(1,25+joint_data)); % qdot_desired
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+6) = (Collision_Aggregate_Data(k-time_step+1,31+joint_data)-MinTrainingData(1,31+joint_data)) / (MaxTrainingData(1,31+joint_data)-MinTrainingData(1,31+joint_data)); % dynamic torque
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+7) = (Collision_Aggregate_Data(k-time_step+1,37+joint_data)-MinTrainingData(1,37+joint_data)) / (MaxTrainingData(1,37+joint_data)-MinTrainingData(1,37+joint_data)); % qabs_desired
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+8) = (Collision_Aggregate_Data(k-time_step+1,43+joint_data)-MinTrainingData(1,43+joint_data)) / (MaxTrainingData(1,43+joint_data)-MinTrainingData(1,43+joint_data)); % qabsdot_desired
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+9) = (Collision_Aggregate_Data(k-time_step+1,55+joint_data)-MinTrainingData(1,55+joint_data)) / (MaxTrainingData(1,55+joint_data)-MinTrainingData(1,55+joint_data)); % temperature
            end
            for ee_data = 1:3
                CollisionProcessData(CollisionProcessDataIdx,(num_data_type-1)*num_time_step*6+3*(time_step-1)+ee_data) = (Collision_Aggregate_Data(k-time_step+1,61+ee_data)-MinTrainingData(1,61+ee_data)) / (MaxTrainingData(1,61+ee_data)-MinTrainingData(1,61+ee_data)); % end effector acceleration
            end
        end
        CollisionProcessDataIdx = CollisionProcessDataIdx +1;
    end
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
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = (Free_Aggregate_Data(k-time_step+1,1+joint_data)-MinTrainingData(1,1+joint_data)) / (MaxTrainingData(1,1+joint_data)-MinTrainingData(1,1+joint_data)); % current
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = (Free_Aggregate_Data(k-time_step+1,7+joint_data)-MinTrainingData(1,7+joint_data)) / (MaxTrainingData(1,7+joint_data)-MinTrainingData(1,7+joint_data)); % q
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+3) = (Free_Aggregate_Data(k-time_step+1,13+joint_data)-MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data)-MinTrainingData(1,13+joint_data)); % qdot
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+4) = (Free_Aggregate_Data(k-time_step+1,19+joint_data)-MinTrainingData(1,19+joint_data)) / (MaxTrainingData(1,19+joint_data)-MinTrainingData(1,19+joint_data)); % q_desired
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+5) = (Free_Aggregate_Data(k-time_step+1,25+joint_data)-MinTrainingData(1,25+joint_data)) / (MaxTrainingData(1,25+joint_data)-MinTrainingData(1,25+joint_data)); % qdot_desired
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+6) = (Free_Aggregate_Data(k-time_step+1,31+joint_data)-MinTrainingData(1,31+joint_data)) / (MaxTrainingData(1,31+joint_data)-MinTrainingData(1,31+joint_data)); % dynamic torque
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+7) = (Free_Aggregate_Data(k-time_step+1,37+joint_data)-MinTrainingData(1,37+joint_data)) / (MaxTrainingData(1,37+joint_data)-MinTrainingData(1,37+joint_data)); % qabs_desired
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+8) = (Free_Aggregate_Data(k-time_step+1,43+joint_data)-MinTrainingData(1,43+joint_data)) / (MaxTrainingData(1,43+joint_data)-MinTrainingData(1,43+joint_data)); % qabsdot_desired
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+9) = (Free_Aggregate_Data(k-time_step+1,55+joint_data)-MinTrainingData(1,55+joint_data)) / (MaxTrainingData(1,55+joint_data)-MinTrainingData(1,55+joint_data)); % temperature
        end
        for ee_data = 1:3
            FreeProcessData(FreeProcessDataIdx,(num_data_type-1)*num_time_step*6+3*(time_step-1)+ee_data) = (Free_Aggregate_Data(k-time_step+1,61+ee_data)-MinTrainingData(1,61+ee_data)) / (MaxTrainingData(1,61+ee_data)-MinTrainingData(1,61+ee_data)); % end effector acceleration
        end
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
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = (Validation_Data(k-time_step+1,1+joint_data)-MinTrainingData(1,1+joint_data)) / (MaxTrainingData(1,1+joint_data)-MinTrainingData(1,1+joint_data)); % current
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = (Validation_Data(k-time_step+1,7+joint_data)-MinTrainingData(1,7+joint_data)) / (MaxTrainingData(1,7+joint_data)-MinTrainingData(1,7+joint_data)); % q
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+3) = (Validation_Data(k-time_step+1,13+joint_data)-MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data)-MinTrainingData(1,13+joint_data)); % qdot
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+4) = (Validation_Data(k-time_step+1,19+joint_data)-MinTrainingData(1,19+joint_data)) / (MaxTrainingData(1,19+joint_data)-MinTrainingData(1,19+joint_data)); % q_desired
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+5) = (Validation_Data(k-time_step+1,25+joint_data)-MinTrainingData(1,25+joint_data)) / (MaxTrainingData(1,25+joint_data)-MinTrainingData(1,25+joint_data)); % qdot_desired
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+6) = (Validation_Data(k-time_step+1,31+joint_data)-MinTrainingData(1,31+joint_data)) / (MaxTrainingData(1,31+joint_data)-MinTrainingData(1,31+joint_data)); % dynamic torque
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+7) = (Validation_Data(k-time_step+1,37+joint_data)-MinTrainingData(1,37+joint_data)) / (MaxTrainingData(1,37+joint_data)-MinTrainingData(1,37+joint_data)); % qabs_desired
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+8) = (Validation_Data(k-time_step+1,43+joint_data)-MinTrainingData(1,43+joint_data)) / (MaxTrainingData(1,43+joint_data)-MinTrainingData(1,43+joint_data)); % qabsdot_desired
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+9) = (Validation_Data(k-time_step+1,55+joint_data)-MinTrainingData(1,55+joint_data)) / (MaxTrainingData(1,55+joint_data)-MinTrainingData(1,55+joint_data)); % temperature
        end
        for ee_data = 1:3
            ValidationProcessData(ValidationProcessDataIdx,(num_data_type-1)*num_time_step*6+3*(time_step-1)+ee_data) = (Validation_Data(k-time_step+1,61+ee_data)-MinTrainingData(1,61+ee_data)) / (MaxTrainingData(1,61+ee_data)-MinTrainingData(1,61+ee_data)); % end effector acceleration
        end
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
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+1) = (Testing_Data(k-time_step+1,1+joint_data)-MinTrainingData(1,1+joint_data)) / (MaxTrainingData(1,1+joint_data)-MinTrainingData(1,1+joint_data)); % current
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+2) = (Testing_Data(k-time_step+1,7+joint_data)-MinTrainingData(1,7+joint_data)) / (MaxTrainingData(1,7+joint_data)-MinTrainingData(1,7+joint_data)); % q
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+3) = (Testing_Data(k-time_step+1,13+joint_data)-MinTrainingData(1,13+joint_data)) / (MaxTrainingData(1,13+joint_data)-MinTrainingData(1,13+joint_data)); % qdot
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+4) = (Testing_Data(k-time_step+1,19+joint_data)-MinTrainingData(1,19+joint_data)) / (MaxTrainingData(1,19+joint_data)-MinTrainingData(1,19+joint_data)); % q_desired
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+5) = (Testing_Data(k-time_step+1,25+joint_data)-MinTrainingData(1,25+joint_data)) / (MaxTrainingData(1,25+joint_data)-MinTrainingData(1,25+joint_data)); % qdot_desired
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+6) = (Testing_Data(k-time_step+1,31+joint_data)-MinTrainingData(1,31+joint_data)) / (MaxTrainingData(1,31+joint_data)-MinTrainingData(1,31+joint_data)); % dynamic torque
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+7) = (Testing_Data(k-time_step+1,37+joint_data)-MinTrainingData(1,37+joint_data)) / (MaxTrainingData(1,37+joint_data)-MinTrainingData(1,37+joint_data)); % qabs_desired
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+8) = (Testing_Data(k-time_step+1,43+joint_data)-MinTrainingData(1,43+joint_data)) / (MaxTrainingData(1,43+joint_data)-MinTrainingData(1,43+joint_data)); % qabsdot_desired
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*(joint_data-1)+(num_data_type-1)*(time_step-1)+9) = (Testing_Data(k-time_step+1,55+joint_data)-MinTrainingData(1,55+joint_data)) / (MaxTrainingData(1,55+joint_data)-MinTrainingData(1,55+joint_data)); % temperature
        end
        for ee_data = 1:3
            TestProcessData(TestProcessDataIdx,(num_data_type-1)*num_time_step*6+3*(time_step-1)+ee_data) = (Testing_Data(k-time_step+1,61+ee_data)-MinTrainingData(1,61+ee_data)) / (MaxTrainingData(1,61+ee_data)-MinTrainingData(1,61+ee_data)); % end effector acceleration
        end
    end
    TestProcessDataIdx = TestProcessDataIdx +1;
end
disp(size(Testing_Data,1))
clear Testing_Data;

csvwrite('TestingData.csv', TestProcessData(1:TestProcessDataIdx-1,:));

