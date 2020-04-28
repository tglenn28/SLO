%_________________________________________________________________________
%   Fixed Load Transformation
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/20/2020
%_________________________________________________________________________


function [edges_gicp_con, edges_gicp_non, cov_gicp_con, cov_gicp_non, edges_sicp_con, edges_sicp_non, cov_sicp_con, cov_sicp_non, T_Pose_GT, edges_sic, edges_gicp] = AliloadTransformations03(file)
load(file);

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



