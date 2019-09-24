clear all
clc
format long

Tool_list = ["0_00kg", "2_01kg", "5_01kg"];

% 충돌

for tool_idx = 1:3
    cd (Tool_list(tool_idx));
    NumCollisionExpFolderName = dir;
    for collision_num =1:size(NumCollisionExpFolderName,1)-2
        cd(int2str(collision_num))
        
        fileName = dir ('DRCL_Data*.txt');
        for l = 1:size(fileName)
            name = fileName(l).name;
            Data = load(name);
            for row_idx =2:size(Data,1)
                Data(row_idx,49) = (Data(row_idx,43)-Data(row_idx-1,43))/0.001;
            end
            save(name, 'Data', '-ascii', '-double', '-tabs')
        end
        cd ..;
    end
    cd ..;
end