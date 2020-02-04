function [array] = getArrayFromCell(cell)
%GETARRAYFROMCELL Function to convert cell of type {{a},{b},{c}} to regular
%array [a, b, c]. This is needed to convert the format of some parameters
%that are loaded from the ROS parameter server.

len = length(cell);

array = zeros(1,len);

for i=1:len
    array(i) = cell2mat(cell(i));
end

end

