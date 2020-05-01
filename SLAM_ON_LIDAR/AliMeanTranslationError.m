%_________________________________________________________________________
%   iSAM on SICP & GICP transformation for Kitti dataset 07
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/30/2020
%_________________________________________________________________________

function [ErrorMean] = AliMeanTranslationError(colors,name,results,gt)

%find vector wide norm between actual and estimate
%results = horzcat([0,0,0;results]);
Enorm = vecnorm(results-gt,2,2);

%find distance metric for results

dist = zeros(size(results,1),1);
for i = 1:size(results,1)
    if i == 1
        dist(i) = norm(results(i,:) - zeros(1,3));
    else
        dist(i) = norm(results(i,:) - results(i-1,:)) + dist(i-1);
    end
    
end

Error = Enorm./dist;


ErrorMean = mean(Error)*100;

figure(2)
hold on
plot(Error,colors,'DisplayName',name);
legend('show');
title('Norm Translation Error');

figure(3)
hold on
plot(Enorm,colors,'DisplayName',name);
legend('show');
title('l2 Norm Error');

figure(4)
hold on
plot(dist(1:end-1),colors,'DisplayName',name);
legend('show');
title('Distance Metric');


end
    
    
    
    