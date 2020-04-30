%_________________________________________________________________________
%	Find Loop Closures and Save
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/30/2020
%_________________________________________________________________________


function closureidx = findloopclosure(gt3, error, write)
gt = [];
for i=1:size(gt3,1)
    pose = [reshape(gt3(i,2:end),4,3)'; 0 0 0 1];
    gt = [gt; pose(1:3,4)'];
end

count = 1;
closureidx = [];
for i = 1:size(gt,1)
    for j= 1:size(gt,1)
    n = vecnorm(gt(i,:)-gt(j,:),2,2);
    
    if n<0.5
       closureidx = [closureidx; j, i];
    end
    end
end

if write == 1
tempText = "loopclosures" + ".mat";
save(tempText, 'closureidx');
end

end












