function M = plotPoses4VideoandCalcError(PoseStruct, iGT, animated,videoName,addPointCloud,PCDDir)
% ------------------------------------------------------------------------
%    plotPoses4VideoandCalcError Function - Creates a simple animation of Poses
%   
%   INPUTS
%   PoseStruct - struct array with two fields - name and poses
%            PoseStruct.name is a char array which gives name of pose to plot
%            PoseStruct.poses is a 17 (or 13) x N Doubles array of N poses 
%            where the transformation matrix is provided in row vector format
%            after the first indice. The first indice identifies the pose #
%   iGT - double scalar that identidies which index of the struct array is
%           the Ground Truth
%   animated - boolean indicated whetehr to animate and save to file or not
%   videoName - VideoName for an animation saved to file
%
%   OUTPUTS
%   M - a struct array with each frame to recreate video if wanted.
%
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 4/29/2020
% ------------------------------------------------------------------------

num_comparison = numel(PoseStruct);
% Set Up Plot Parameters
f = figure('Units','normalized','OuterPosition',[0 .25 .75 .75],'Color','w');
ax1 = axes('Parent',f); axis tight; axis equal; view(ax1,-180,0);

%ax2 = subplot(2,1,2); axis tight; %axis equal;
f2 = figure('Units','normalized','OuterPosition',[0 .25 0.885 .5],'Color','w');
ax3 = axes('Parent',f2); 
hold(ax1,'on');  hold(ax3,'on'); 
grid(ax1,'on');  grid(ax3,'on'); 

% Set UP Lidar Plots 
f3 = figure('Visible','off');
ax5 = axes('Parent',f3);

% Initialize Number of Plot Objects based on Size of Struct
pHandlesTrajectory1 = gobjects(num_comparison,1);

pHandlesError = gobjects(2,1);


% Which is my GT
GT_Poses = PoseStruct(iGT).poses;
% Calculate Cummulative DIstance
GT_cumm_dist = 0;
pt_GT = [GT_Poses(1,5) GT_Poses(1,9) GT_Poses(1,13)];
for i=2:size(GT_Poses,1)
    
    pt_GT = [pt_GT; [GT_Poses(i,5) GT_Poses(i,9) GT_Poses(i,13)]];

    GT_cumm_dist = [GT_cumm_dist; sum(vecnorm(pt_GT(1:i-1,:)' - pt_GT(2:i,:)'))];
end

allTranslationErrors = NaN(size(PoseStruct(iGT).poses,1),num_comparison-2);
allTransformationError = NaN(size(PoseStruct(iGT).poses,1),num_comparison-2);
for jPose = 1:size(PoseStruct(iGT).poses,1)
   

    for iComparePose=1:num_comparison
        tmp_pose = PoseStruct(iComparePose).poses;
        
        if jPose == min(tmp_pose(:,1)) + 1 % Drop first Point and Create Object to Update
            
             
            if iComparePose > 2 
                % Store Moving Translation Error for Later
                allTranslationErrors(jPose,iComparePose-2) = vecnorm([GT_Poses(jPose,5) GT_Poses(jPose,9) GT_Poses(jPose,13)]' - ...
                    [tmp_pose(jPose,5) tmp_pose(jPose,9) tmp_pose(jPose,13)]')'/GT_cumm_dist(jPose);
   
                
                % Store Distance Metric - Transformation Error
                T1 = inv([reshape(GT_Poses(jPose,2:end),4,3)'; 0 0 0 1])*[reshape(GT_Poses(jPose+1,2:end),4,3)'; 0 0 0 1];
                T2 = inv(reshape(tmp_pose(jPose,2:end),4,4)')*reshape(tmp_pose(jPose+1,2:end),4,4)';
                T1_T2 = logm(T1\T2);
                allTransformationError(jPose,iComparePose-2) = norm(T1_T2(:)); %/GT_cumm_dist(jPose);
                
            end
            % Plot Two Views of TRajectory
            pHandlesTrajectory1(iComparePose) = plot3(ax1,tmp_pose(jPose,5),tmp_pose(jPose,9),tmp_pose(jPose,13),...
                'DisplayName',PoseStruct(iComparePose).name,'LineWidth',2);
            
            legend(ax1,'show');
            
%             pHandlesTrajectory2(iComparePose) = plot3(ax2,tmp_pose(jPose,5),tmp_pose(jPose,9),tmp_pose(jPose,13),...
%                 'DisplayName',PoseStruct(iComparePose).name);
%             legend(ax2,'show');
            
        elseif jPose > min(tmp_pose(:,1)) + 1 && jPose <= max(tmp_pose(:,1)) + 1
            start = min(tmp_pose(:,1)) + 1;
            % Store Moving Error for Later
            if iComparePose > 2
                
                allTranslationErrors(jPose,iComparePose-2) = vecnorm([GT_Poses(jPose,5) GT_Poses(jPose,9) GT_Poses(jPose,13)]' - ...
                    [tmp_pose(jPose,5) tmp_pose(jPose,9) tmp_pose(jPose,13)]')'/GT_cumm_dist(jPose);
               
                if jPose ~= max(tmp_pose(:,1)) + 1
                    % Store Distance Metric - Transformation Error
                    T1 = inv([reshape(GT_Poses(jPose,2:end),4,3)'; 0 0 0 1])*[reshape(GT_Poses(jPose+1,2:end),4,3)'; 0 0 0 1];
                    T2 = inv(reshape(tmp_pose(jPose,2:end),4,4)')*reshape(tmp_pose(jPose+1,2:end),4,4)';
                    T1_T2 = logm(T1\T2);
                    allTransformationError(jPose,iComparePose-2) = norm(T1_T2(:)); %/GT_cumm_dist(jPose);
                end
            end
            title(ax1,{'07 Trajectory',strcat("Pose: ",num2str(jPose))});
            title(ax3,{'Difference in Transformation Error','T1: Moving Classes Removed - T2: Moving Classes Maintained',strcat("Pose: ",num2str(jPose))});
            % Update Object with new Point
            set(pHandlesTrajectory1(iComparePose),'XData',tmp_pose(start:jPose,5),'YData',...
                tmp_pose(start:jPose,9),'ZData',tmp_pose(start:jPose,13));
                         
            
            
            
        end
        
    end

    d3 = allTransformationError(1:jPose,3) - allTransformationError(1:jPose,1);
    d4 = allTransformationError(1:jPose,4) - allTransformationError(1:jPose,2);
    if jPose == min(tmp_pose(:,1)) + 1
        
        
        pHandlesError(1) = plot(ax3,d3,'DisplayName','GICP','LineWidth',2);
        pHandlesError(2) = plot(ax3,d4,'DisplayName','SICP','LineWidth',2);
        legend(ax3,'show');
    elseif jPose > min(tmp_pose(:,1)) + 1 && jPose <= max(tmp_pose(:,1)) + 1

        set(pHandlesError(1),'YData',d3);
        set(pHandlesError(2),'YData',d4);
    end           

    % If Animated We Must Save Frames for a Video
    if animated
        drawnow
        M(jPose) = getframe(f);
        M2(jPose)= getframe(f2);
    else 
        M = [];
    end
    % If Add Point Cloud Attempt to PLot PCD Point Clouds
    if addPointCloud
        pcshow(PCDDir{jPose},'Parent',ax5);
        drawnow
    end
    %Adjust Axes of PLot 2 to 
    
    
    square = 200;
    if jPose > square

        xlim(ax3,[jPose - square jPose + 5]);
    end

end
%M = [];

%%  Save a Video
if animated
    % create the video writer with 10 fps
    videoName = strcat(videoName,'.mp4');
    writerObj1 = VideoWriter(videoName,'MPEG-4');
    writerObj1.FrameRate = 10; % set the seconds per image
    
    videoName2 = strcat(videoName,'_Error','.mp4');
    writerObj2 = VideoWriter(videoName2,'MPEG-4');
    writerObj2.FrameRate = 10; % set the seconds per image
    
    % open the video writer
    open(writerObj1);
    open(writerObj2);
    % write the frames to the video
    for i=1:length(M)
        % convert the image to a frame
        frame = M(i);
        frame2 = M2(i);
        writeVideo(writerObj1, frame);
        writeVideo(writerObj2, frame2);
    end
    % close the writer object
    close(writerObj1);
    close(writerObj2);
end
end