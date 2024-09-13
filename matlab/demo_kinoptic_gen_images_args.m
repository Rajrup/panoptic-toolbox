% A demo to show a simple example to generate pointcloud generation 
% from 10 Kinect depth maps of Kipnoptic studio
%
% Note: the output point cloud is located in the same coordinate of
% panoptic studio (480 VGAs + 31 HDs) where 3D skeletons are defined. 
%
% Note2: Frame indices of output ply files are consistent to HD frames of the
% Panoptic Studio. That is ptcloud_000000001.ply is the first frame of HD videos
%
% Note3: This code assumes that your matalb has Computer Vision toolbox to
% save and visualize point cloud data
% (https://www.mathworks.com/help/vision/3-d-point-cloud-processing.html)
%
% Hanbyul Joo (hanbyulj@cs.cmu.edu) and Tomas Simon (tsimon@cs.cmu.edu)

% Input/Output Path Setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following folder hiearchy is assumed:
% (root_path)/(seqName)/kinect_shared_depth
% (root_path)/(seqName)/kinectImgs
% (root_path)/(seqName)/kcalibration_(seqName).json
% (root_path)/(seqName)/ksynctables_(seqName).json
% (root_path)/(seqName)/calibration_(seqName).json
% (root_path)/(seqName)/synctables_(seqName).json

% demo_kinoptic_gen_images_args('160422_haggling1', 1, 14000)
% demo_kinoptic_gen_images_args('160906_ian3', 1, 9000)
% demo_kinoptic_gen_images_args('170307_dance5', 1, 12000)
% demo_kinoptic_gen_images_args('170915_office1', 1, 7000)
% demo_kinoptic_gen_images_args('170915_toddler4', 1, 6000)
% demo_kinoptic_gen_images_args('160906_pizza1', 1, 10000)
% demo_kinoptic_gen_images_args('160906_band2', 1, 10000)
% demo_kinoptic_gen_images_args('160224_ultimatum2', 1, 16000)
% demo_kinoptic_gen_images_args('160226_mafia1', 1, 20000)

function demo_kinoptic_gen_images_args(seq_name, start_id, end_id)
    root_path = '/data/panoptic-toolbox/'; %Put your root path where sequence folders are locates
    % seqName = '160422_haggling1'  %Put your target sequence name here
    seqName = seq_name;
    hd_index_list= start_id:end_id; % Target frames you want to export ply files

    %The followings are an example
    % root_path = '/posefs0c/panoptic' %An example
    % seqName = '160422_haggling1' %An example
    % hd_index_list= 500:510; % Target frames you want to export ply files

    %Relative Paths
    kinectImgDir = sprintf('%s/%s/kinectImgs',root_path,seqName);  
    kinectDepthDir = sprintf('%s/%s/kinect_shared_depth',root_path,seqName);
    calibFileName = sprintf('%s/%s/kcalibration_%s.json',root_path,seqName,seqName);
    syncTableFileName = sprintf('%s/%s/ksynctables_%s.json',root_path,seqName,seqName);
    panopcalibFileName = sprintf('%s/%s/calibration_%s.json',root_path,seqName,seqName);
    panopSyncTableFileName = sprintf('%s/%s/synctables_%s.json',root_path,seqName,seqName);
    
    rgbImgOutDir = sprintf('%s/%s/kinectRGBImgs',root_path,seqName);
    depthImgOutDir = sprintf('%s/%s/kinectDepthImgs',root_path,seqName);
    rawDepthImgOutDir = sprintf('%s/%s/kinectRawDepthImgs',root_path,seqName);
    mkdir(rgbImgOutDir);
    mkdir(depthImgOutDir);
    %mkdir(rawDepthImgOutDir);

    SyncOutputFile= fopen(sprintf('%s/%s/sync_table_gen_images_matlab.txt', root_path, seqName),'w');

    % Output folder Path
    %Change the following if you want to save outputs on another folder
    plyOutputDir=sprintf('%s/%s/kinoptic_ptclouds',root_path,seqName);
    mkdir(plyOutputDir);
    disp(sprintf('PLY files will be saved in: %s',plyOutputDir));

    %Other parameters
    bVisOutput = 0; %Turn on, if you want to visualize what's going on
    bRemoveFloor= 0;  %Turn on, if you want to remove points from floor
    floorHeightThreshold = 0.5; % Adjust this (0.5cm ~ 7cm), if floor points are not succesfully removed
                                % Icreasing this may remove feet of people
    bRemoveWalls = 0; %Turn on, if you want to remove points from dome surface

    addpath('jsonlab');
    addpath('kinoptic-tools');

    %% Load syncTables
    ksync = loadjson(syncTableFileName);
    knames = {};
    for id=1:10; knames{id} = sprintf('KINECTNODE%d', id); end

    psync = loadjson(panopSyncTableFileName); %%Panoptic Sync Tables


    %% Load Kinect Calibration File
    kinect_calibration = loadjson(calibFileName); 

    panoptic_calibration = loadjson(panopcalibFileName);
    panoptic_camNames = cellfun( @(X) X.name, panoptic_calibration.cameras, 'uni', false ); %To search the targetCam


    hd_index_list = hd_index_list+2; %This is the output frame (-2 is some weired offset in synctables)

    for hd_index = hd_index_list

        hd_index_afterOffest = hd_index-2; %This is the output frame (-2 is some weired offset in synctables)

    %     if( mod(hd_index_afterOffest,10)~=0)
    %         continue;       %We ALWAYS save every 10th frames.
    %     end
        out_fileName = sprintf('%s/ptcloud_hd%08d.ply', plyOutputDir, hd_index_afterOffest);

    %     if exist(out_fileName,'file')
    %         disp(sprintf('Already exists: %s\n',out_fileName));
    %         continue;
    %     end

        %% Compute Universal time
        selUnivTime = psync.hd.univ_time(hd_index);
        fprintf('hd_index: %d, UnivTime: %.3f\n', hd_index, selUnivTime)


        %% Main Iteration    
        all_point3d_panopticWorld = []; %point cloud output from all kinects
        all_colorsv = [];   %colors for point cloud 


        for idk = 1:10  %Iterating 10 kinects. Change this if you want a subpart

            if idk==1 && bVisOutput   %Visualize the results from the frist kinect only. 
                vis_output = 1;
            else
                vis_output = 0;
            end

            %% Select corresponding frame index rgb and depth by selUnivTime
            % Note that kinects are not perfectly synchronized (it's not possible),
            % and we need to consider offset from the selcUnivTime
            [time_distc, cindex] = min( abs( selUnivTime - (ksync.kinect.color.(knames{idk}).univ_time-6.25) ) );  %cindex: 1 based
            ksync.kinect.color.(knames{idk}).univ_time(cindex);
            % assert(time_dist<30);
            [time_distd, dindex] = min( abs( selUnivTime - ksync.kinect.depth.(knames{idk}).univ_time ) ); %dindex: 1 based

            % Filtering if current kinect data is far from the selected time
            fprintf('idk: %d, %.4f\n', idk, selUnivTime - ksync.kinect.depth.(knames{idk}).univ_time(dindex));
            if abs(ksync.kinect.depth.(knames{idk}).univ_time(dindex) - ksync.kinect.color.(knames{idk}).univ_time(cindex))>6.5
                fprintf('Skipping %d, depth-color diff %.3f\n',  abs(ksync.kinect.depth.(knames{idk}).univ_time(dindex) - ksync.kinect.color.(knames{idk}).univ_time(cindex)));    
                continue;
            end
            % assert(time_dist<30);
            % time_distd
            if time_distc>30 || time_distd>17 
                fprintf('Skipping %d\n', idk);
                [time_distc, time_distd];
                continue;
            end

            % Extract image and depth
            %rgbim_raw = kdata.vobj{idk}.readIndex(cindex); % cindex: 1 based
            rgbFileName = sprintf('%s/50_%02d/50_%02d_%08d.png',kinectImgDir,idk,idk,cindex);
            depthFileName = sprintf('%s/KINECTNODE%d/depthdata.dat',kinectDepthDir,idk);

            rgbim = imread(rgbFileName); % cindex: 1 based
            %depthim_raw = kdata.vobj{idk}.readDepthIndex(dindex);  % cindex: 1 based
            depthim = readDepthIndex_1basedIdx(depthFileName,dindex);  % cindex: 1 based

            %Check valid pixels
            validMask = depthim~=0; %Check non-valid depth pixels (which have 0)
            nonValidPixIdx = find(validMask(:)==0);
            %validPixIdx = find(validMask(:)==1);

            if vis_output
                figure; imshow(rgbim);     title('RGB Image');
                figure; imagesc(depthim);  title('Depth Image');
                figure; imshow(validMask*255); title('Validity Mask');
            end
            
            %rawDepthFileNameOut = sprintf('%s/50_%02d_%08d.png', rawDepthImgOutDir,idk,dindex);
            %imwrite(uint16(depthim), rawDepthFileNameOut);
            


            %% Back project depth to 3D points (in camera coordinate)
            camCalibData = kinect_calibration.sensors{idk};

            % point3d (N x 3): 3D point cloud from depth map in the depth camera cooridnate
            % point2d_color (N x 2): 2D points projected on the rgb image space
            % Where N is the number of pixels of depth image (512*424)
            [point3d, point2d_incolor] = unprojectDepth_release(depthim, camCalibData, true);


            validMask = validMask(:) &  ~(point3d(:,1)==0);
            nonValidPixIdx = find(validMask(:)==0);


%             point3d(nonValidPixIdx,:) = nan;
%             point2d_incolor(nonValidPixIdx,:) = nan;

    %         if vis_output
    %             figure; plot3(point3d(:,1),point3d(:,2),point3d(:,3),'.'); axis equal;  %Plot raw 3D points
    %         end

            %% Filtering based on the distance from the dome center
            domeCenter_kinectlocal = camCalibData.domeCenter;

            if bRemoveWalls
                dist = sqrt(sum(bsxfun(@minus, point3d(:,1:3), domeCenter_kinectlocal').^2,2));
                point3d(dist>2.5,:) = nan;
                point3d(point3d(:,2)>2.3,:) = nan;
            end

            if vis_output
                figure; plot3(point3d(:,1),point3d(:,2),point3d(:,3),'.'); axis equal;  %Plot raw 3D points
                title('Unprojecting Depth from Kinect1 (after filtering dome wall');
                view(2);
            end


            %% Project 3D points (from depth) to color image
            colors_inDepth = multiChannelInterp( double(rgbim)/255, ...
                point2d_incolor(:,1)+1, point2d_incolor(:,2)+1, 'linear');
            
%             colors_inDepth_list_u8 = uint8(colors_inDepth * 255.0);

            colors_inDepth = reshape(colors_inDepth, [size(depthim,1), size(depthim,2), 3]);
            colorsv = reshape(colors_inDepth, [], 3);

            colors_inDepth_u8 = uint8(colors_inDepth * 255.0);
            colorsv_u8 = uint8(colorsv * 255.0);
            
            rgbImgOutSubDir = sprintf('%s/50_%02d',rgbImgOutDir,idk);
            depthImgOutSubDir = sprintf('%s/50_%02d',depthImgOutDir,idk);
            if ~exist(rgbImgOutSubDir, 'dir')
                mkdir(rgbImgOutSubDir);
            end
            if ~exist(depthImgOutSubDir, 'dir')
                mkdir(depthImgOutSubDir);
            end

            rgbFileNameOut = sprintf('%s/50_%02d_%08d_transformed.png', rgbImgOutSubDir,idk,cindex);
            depthFileNameOut = sprintf('%s/50_%02d_%08d_transformed.bin', depthImgOutSubDir,idk,dindex);
            
            colors_u8 = uint8(colors_inDepth*255.0);
            point3d_float = single(point3d);
            point3d_float(isnan(point3d_float)) = single(0);

            imwrite(colors_u8, rgbFileNameOut);
            fileID = fopen(depthFileNameOut,'w');
            fwrite(fileID, point3d_float', 'float');
            fclose(fileID);
            fprintf('RGB file: %s\n', rgbFileNameOut);
            fprintf('Depth file: %s\n', depthFileNameOut);

            fprintf(SyncOutputFile, 'SYNC - HD Index: %d, Cam ID: %d, Color Index: %d, Depth Index: %d\n', hd_index, idk, cindex, dindex);
            
        end
    end
end

