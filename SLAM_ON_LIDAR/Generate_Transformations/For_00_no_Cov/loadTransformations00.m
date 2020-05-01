function [T_SICP, T_GICP] = loadTransformations00(file)
    transforms = load(file);
    T_SICP = transforms(1:4,:);
    T_GICP = transforms(5:8,:);
end