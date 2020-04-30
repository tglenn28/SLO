%_________________________________________________________________________
%   Fixed Load Transformation
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/30/2020
%_________________________________________________________________________


function [edges_gicp_con, edges_gicp_non, cov_gicp_con, cov_gicp_non, edges_sicp_con, edges_sicp_non, cov_sicp_con, cov_sicp_non, T_Pose_GT, edges_sic, edges_gicp] = AliloadTransformations03(file)
load(file);

%% Apply transformations on data to bring to correct frame
% Tr =   [2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
%     1.044940741659e-02 1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
%     9.999453885620e-01 1.243653783865e-04 1.045130299567e-02 -2.721327964059e-01;
%     0                   0                   0                   1];
% 
% import gtsam.*
% for i = 1:length(edges_sic)
% pose3 = (Tr * reshape(edges_sic(i,3:18),4,4)') / Tr;
% 
% edges_sic(i,3:18) = pose3(:)';
% end
% 
% for i = 1:length(edges_gicp)
% pose3 = (Tr * reshape(edges_gicp(i,3:18),4,4)') / Tr;
% 
% edges_gicp(i,3:18) = pose3(:)';
% end

%%  GICP data 
diff = edges_gicp(:,1)-edges_gicp(:,2);
test = horzcat(edges_gicp,diff);
cov_test = horzcat(covar,diff);

sort = sortrows(test,size(test,2));
cov_sort = sortrows(cov_test,size(cov_test,2));

sort = sort(sort(:,size(test,2))>=0,:);
cov_sort = cov_sort(cov_sort(:,size(cov_test,2))>=0,:);

edges_gicp_sort = sort;
edges_gicp_con = sort(sort(:,size(test,2))==1,:);
edges_gicp_non = sort(sort(:,size(test,2))>1,:);

cov_sort = cov_sort;
cov_gicp_con = cov_sort(cov_sort(:,size(cov_test,2))==1,:);
cov_gicp_non = cov_sort(cov_sort(:,size(cov_test,2))>1,:);

%%  SICP data 

diff = edges_sic(:,1)-edges_sic(:,2);
test = horzcat(edges_sic,diff);
cov_test = horzcat(covar,diff);

sort = sortrows(test,size(test,2));
cov_sort = sortrows(cov_test,size(cov_test,2));

sort = sort(sort(:,size(test,2))>=0,:);
cov_sort = cov_sort(cov_sort(:,size(cov_test,2))>=0,:);

edges_sicp_sort = sort;
edges_sicp_con = sort(sort(:,size(test,2))==1,:);
edges_sicp_non = sort(sort(:,size(test,2))>1,:);

cov_sort = cov_sort;
cov_sicp_con = cov_sort(cov_sort(:,size(cov_test,2))==1,:);
cov_sicp_non = cov_sort(cov_sort(:,size(cov_test,2))>1,:);


end



