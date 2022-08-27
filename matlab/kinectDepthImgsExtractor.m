function extract_depth_imgs(arg1)
    root_path = '/bigdata/rajrup/Dataset_Repo/panoptic-toolbox/data' %Put your root path where sequence folders are locates
    % seqName = '160422_haggling1'  %Put your target sequence name here
    seqName = arg1;
    hd_index_list= 1:13750; % Target frames you want to export ply files

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
    
    % Output folder Path
    %Change the following if you want to save outputs on another folder
    depthImgOutputDir=sprintf('%s/%s/kinopticDepthImgs',root_path,seqName);
%     mkdir(plyOutputDir);
    disp(sprintf('Depth Images will be saved in: %s',depthImgOutputDir));
    
    %Other parameters
    vis_output = 0; %Turn on, if you want to visualize what's going on
    
    addpath('jsonlab');
    addpath('kinoptic-tools');
    
    %% Load syncTables
    ksync = loadjson(syncTableFileName);
    knames = {};
    for id=1:10; knames{id} = sprintf('KINECTNODE%d', id); end

    psync = loadjson(panopSyncTableFileName); %%Panoptic Sync Tables
    
    hd_index_list = ksync.kinect.depth.(knames{1}).index + 1;

    for hd_index = hd_index_list
        fprintf('hd idx: %d\n', hd_index);
        for idk = 1:10  %Iterating 10 kinects. Change this if you want a subpart
            %% Extract image and depth
            %rgbim_raw = kdata.vobj{idk}.readIndex(cindex); % cindex: 1 based
            depthImgIdkOutputDir = sprintf('%s/50_%02d',depthImgOutputDir,idk);
            if ~exist(depthImgIdkOutputDir, 'dir')
                mkdir(depthImgIdkOutputDir);
            end
            
            rgbFileName = sprintf('%s/50_%02d/50_%02d_%08d.jpg',kinectImgDir,idk,idk,hd_index);
            depthFileName = sprintf('%s/KINECTNODE%d/depthdata.dat',kinectDepthDir,idk);
            depthImgFileNameOutput = sprintf('%s/50_%02d_%08d.png', depthImgIdkOutputDir,idk,hd_index);

            rgbim = imread(rgbFileName);
            %depthim_raw = kdata.vobj{idk}.readDepthIndex(dindex);  % cindex: 1 based
            depthim = readDepthIndex_1basedIdx(depthFileName, hd_index);
            
            %Check valid pixels
            validMask = depthim~=0; %Check non-valid depth pixels (which have 0)
            nonValidPixIdx = find(validMask(:)==0);
            %validPixIdx = find(validMask(:)==1);

            if vis_output
                figure; imshow(rgbim);     title('RGB Image');
                figure; imagesc(depthim);  title('Depth Image');
                figure; imshow(validMask*255); title('Validity Mask');
            end
            
            imwrite(depthim, depthImgFileNameOutput);
            
        end
    end
end