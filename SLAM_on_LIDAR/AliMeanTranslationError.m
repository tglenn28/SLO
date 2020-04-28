function [ErroMean] = AliMeanTranslationError(results,gt)

    %find vector wide norm between actual and estimate
    Enorm = vecnorm(results-gt,2,2);
    
    %find distance metric for results
    dist = [];
    dist(1) = 0;
    for i = 2:size(results,1)-1
    dist(i) = norm(results(i-1)-results(i))+dist(i-1);
    end
    
    Error = Enorm/dist;
    
    ErrorMean = mean(Error);
    
end
    
    
    
    