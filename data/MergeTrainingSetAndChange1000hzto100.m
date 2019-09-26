clear all
clc
format long

%% Load Robot Model
DoosanMSeries_RoboticsToolBox_Simulator;
Tool_list = ["0_00kg", "2_01kg", "5_01kg"];

%% 날짜별
FolderName = dir;
folder_idx = 1;
for time_step = 1:size(FolderName,1)
    if (size(FolderName(time_step).name,2) > 5) && strcmp(FolderName(time_step).name(1:6), 'robot1')
        DataFolderList(folder_idx) = string(FolderName(time_step).name);
        folder_idx = folder_idx + 1;
    end
end

%% 충돌
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
                    Collision_Aggregate_Data = [];
                    cd(int2str(collision_num))
                    fileName = dir ('DRCL_Data*.txt');
                    for l = 1:size(fileName)
                        Data = load(fileName(l).name);
                        Collision_Aggregate_Data = vertcat(Collision_Aggregate_Data, Data);
                    end
                    
                    ReducedData = zeros(fix(size(Collision_Aggregate_Data,1)/10), size(Collision_Aggregate_Data,2));
                    for row = 1:size(ReducedData,1)
                        ReducedData(row,:) = Collision_Aggregate_Data(10*(row-1)+1,:);
                    end
                    
                    % 가속도 보상
                    AngleM = ReducedData(:,8:13);
                    VelD = ReducedData(:,26:31);
                    ToolAccM = ReducedData(:,62:64);
                    
                    ToolGrav = zeros(size(ReducedData,1),3);
                    ToolAccD = zeros(size(ReducedData,1),3);
                    BaseAccD = zeros(size(ReducedData,1),6);
                    for m = 1:size(ReducedData,1)
                        R_T = bot.fkine(AngleM(m,:)).R()';
                        ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
                        if m == 1
                            BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m+1,:) - VelD(m,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
                        else
                            BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m,:) - VelD(m-1,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
                        end
                        ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
                        if mod(m, 10000) == 0
                            m
                        end
                    end
                    ReducedData(:,62:64) = ToolAccM+ToolGrav-ToolAccD;

                    save('Reduced_DRCL_Data.txt', 'ReducedData', '-ascii', '-double', '-tabs')
                    cd ..;
                end
                cd ..;
            end
            cd ..;
        end
    end
    cd ..;
end


%% 자유모션
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
                    Free_Aggregate_Data = [];
                    cd(int2str(collision_num))
                    fileName = dir ('DRCL_Data*.txt');
                    for l = 1:size(fileName)
                        Data = load(fileName(l).name);
                        Free_Aggregate_Data = vertcat(Free_Aggregate_Data, Data);
                    end
                    
                    ReducedData = zeros(fix(size(Free_Aggregate_Data,1)/10), size(Free_Aggregate_Data,2));
                    for row = 1:size(ReducedData,1)
                        ReducedData(row,:) = Free_Aggregate_Data(10*(row-1)+1,:);
                    end
                    
                    % 가속도 보상
                    AngleM = ReducedData(:,8:13);
                    VelD = ReducedData(:,26:31);
                    ToolAccM = ReducedData(:,62:64);
                    
                    ToolGrav = zeros(size(ReducedData,1),3);
                    ToolAccD = zeros(size(ReducedData,1),3);
                    BaseAccD = zeros(size(ReducedData,1),6);
                    for m = 1:size(ReducedData,1)
                        R_T = bot.fkine(AngleM(m,:)).R()';
                        ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
                        if m == 1
                            BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m+1,:) - VelD(m,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
                        else
                            BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m,:) - VelD(m-1,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
                        end
                        ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
                        if mod(m, 10000) == 0
                            m
                        end
                    end
                    ReducedData(:,62:64) = ToolAccM+ToolGrav-ToolAccD;
                    
                    save('Reduced_DRCL_Data.txt', 'ReducedData', '-ascii', '-double', '-tabs')
                    cd ..;
                end
                cd ..;
            end
            cd ..;
        end
    end
    cd ..;
end
