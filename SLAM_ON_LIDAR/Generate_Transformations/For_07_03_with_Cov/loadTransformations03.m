function [T_SICP, T_GICP, cov, MSE] = loadTransformations03(file)
    transforms =  importdata(file, " ", 1);
    T_SICP = reshape(transforms.data(end-31:end-16),4,4)';
    T_GICP = reshape(transforms.data(end-15:end),4,4)';
    cov = reshape(transforms.data(1:36),6,6);
    
    MSE = str2double(erase(transforms.textdata, "MSE: "));
end