clear all
clc
format long

TestData = load('DRCL_Data_Test.txt');
ReducedData = zeros(fix(size(TestData,1)/10), size(TestData,2));
for row = 1:size(ReducedData,1)
    ReducedData(row,:) = TestData(10*(row-1)+1,:);
end
save('Reduced_DRCL_Data_Test.txt', 'ReducedData', '-ascii', '-double', '-tabs')