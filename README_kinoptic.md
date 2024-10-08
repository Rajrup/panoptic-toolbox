# KinopticStudio Toolbox

Kinoptic Studio is a subsystem of Panoptic Studio, which is composed of 10 Kinect2 sensors. 

Kinoptic Studio can be independently used from the Panoptic Studio.

See our [PtCloudDB document](http://domedb.perception.cs.cmu.edu/ptclouddb.html) for more details

# Quick start guide
Follow these steps to set up a simple example:


## 1. Download a data

Assuming you want to donwload a sequence: named "160422_haggling1"
```
./scripts/getData_kinoptic.sh 160422_haggling1
```

This script will download the following files. 

* 160422_haggling1/kinect_shared_depth/ksynctables.json   #sync table
* 160422_haggling1/kinect_shared_depth/KINECTNODE%d/depthdata.dat  #depth files
* 160422_haggling1/kcalibration_160422_haggling1.json #multiple kinects calibration files
* 160422_haggling1/kinectVideos/kinect_50_%d.mp4 #rgb video files

## 2. Extract RGB+Depth frames

Step 1: Extract RGB+Depth frames from the downloaded data
```
cd 160422_haggling1
../scripts/kinectImgsExtractor.sh       # extracts RGB in .jpg format
../scripts/kinectImgsExtractor.sh png   # extracts RGB in .png format 
```

Step 2: Extract RGB+Depth frames required for the C++ pipeline 
Run the script `matlab/demo_kinoptic_gen_images_args.m` in matlab to extract RGB and depth frames for the C++ pipeline.
```
demo_kinoptic_gen_images_args('160422_haggling1')
```
## 3. Run demo to generate point clouds from 10 kinects

```
matlab ./matlab/demo_kinoptic_gen_ptcloud.m
```

Note that you should set your "root_path" and "seqName" in this demo file. 



## 4. Run demo to project point clouds on a HD view

```
matlab ./matlab/demo_kinoptic_projection.m
```

Similarly, note that you should set your "root_path" and "seqName" in this demo file. 
