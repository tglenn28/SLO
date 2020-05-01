%_________________________________________________________________________
%   Fixed Load Transformation
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/30/2020
%_________________________________________________________________________


function [edges_gicp_sort, edges_gicp_con, edges_gicp_non, edges_sicp_sort, edges_sicp_con, edges_sicp_non, T_Pose_GT, edges_sic, edges_gicp] = AliloadTransformations(file)
load(file);

diff = edges_gicp(:,1)-edges_gicp(:,2);
test = horzcat(edges_gicp,diff);
sort = sortrows(test,19);
sort = sort(sort(:,19)>=0,:);

edges_gicp_sort = sort;
edges_gicp_con = sort(sort(:,19)==1,:);
edges_gicp_non = sort(sort(:,19)>1,:);

diff = edges_sic(:,1)-edges_sic(:,2);
test = horzcat(edges_sic,diff);
sort = sortrows(test,19);
sort = sort(sort(:,19)>=0,:);

edges_sicp_sort = sort;
edges_sicp_con = sort(sort(:,19)==1,:);
edges_sicp_non = sort(sort(:,19)>1,:);
end



