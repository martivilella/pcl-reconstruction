% https://www.mathworks.com/help/vision/ug/point-cloud-registration-workflow.html

%% Global vars
vSet = pcviewset;
start_pcl = 1; % filename idx number, 1-based (0000 file not supported)
end_pcl = 100;
ext = '.pcd';

%% Import, Downsample, Pairwise registration
% initial pcl
pclid = pad(string(start_pcl),4,'left','0'); % use 1-based index for compatibility with viewId in pcviewset
pcl = pcdenoise(pcread(strcat(pclid,ext)));
pcl_down = pcdownsample(pcl, 'gridAverage', 0.01);
absPose = rigid3d;
vSet = addView(vSet, start_pcl, absPose, 'PointCloud', pcl);
fixed = pcl_down;

% iterate over rest of points
for i=(start_pcl+1):end_pcl
    fprintf('Propagating odometry on idx %i\n', i)
    % load moving (next) pcl
    pclid = pad(string(i),4,'left','0');
    pcl = pcdenoise(pcread(strcat(pclid,ext)));
    pcl_down = pcdownsample(pcl, 'gridAverage', 0.01);
    moving = pcl_down;
    
    % pairwise registration
    relPose = icp_method(moving, fixed);
    absPose = rigid3d(absPose.T*relPose.T);
    vSet = addView(vSet, i, absPose, 'PointCloud', pcl);
    vSet = addConnection(vSet, (i-1), i, relPose);
    
    % update vars for next pair
    fixed = moving;
end

%% Global Loop Closure connectors using ContextDescriptors
for i=start_pcl:end_pcl
    for j=start_pcl:end_pcl
        if abs(i-j)>1 % if they are not neighbours (or the same)
            fixed = pcdownsample(vSet.Views.PointCloud(i-start_pcl+1), 'gridAverage', 0.01);
            moving = pcdownsample(vSet.Views.PointCloud(j-start_pcl+1), 'gridAverage', 0.01);
            descriptor1 = scanContextDescriptor(fixed);
            descriptor2 = scanContextDescriptor(moving);
            dist = scanContextDistance(descriptor1, descriptor2);
            if (dist < 0.1) % potential loop closure
                [relPose,rmse] = icp_method(moving, fixed);
                if rmse < 0.04
                    fprintf('LC edge: RMSE=%4.2f m, CtxtDist=%3.2f, idx pair=(%i,%i)\n', rmse, dist, i, j)
                    vSet = addConnection(vSet, i, j, relPose);
                end
            end
        end
    end
end


%% Optimise poses based on LC edges
vSetOptim = optimizePoses(vSet);

%% Assemble map
gridStep = 0.001;
ptCloudMap = pcalign(vSetOptim.Views.PointCloud, vSetOptim.Views.AbsolutePose, gridStep);

%% Show map
pcshow(ptCloudMap)

%% Write to file
pcwrite(ptCloudMap, strcat('OUTreconstructedScene', ext))

%% Defs

function [relPose,rmse] = icp_method(moving, fixed)
    [relPose,~,rmse] = pcregistericp(moving, fixed, 'Metric', 'pointToPoint', ...
        'MaxIterations', 75, 'Tolerance', [0.001, 0.01], 'Extrapolate', true);
end
