clear all
clc
format long

Tool_list = ["0_00kg", "2_01kg", "5_01kg"];




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
