# Udacity Self-Driving Car Engineer Nanodegree Localization Project

### Purpose of this repo
In this project, students will recover the position of a simulated car using lidar with either ICP or NDT, two scan matching algorithms, aligning point cloud scans from the CARLA simulator. Students will need to achieve sufficient accuracy for the entirety of a drive within the simulated environment, updating the vehicle’s location appropriately as it moves and obtains new lidar data.

This repo contains completed codes for this project.


### Build

Warning: Some libraries deleted for decreasing file size. For all files, please check course workspace.


```
cd /home/workspace/c3-project
cmake .
make
```


### Run
```
su - student // Ignore Permission Denied, if you see student@ you are good
cd /home/workspace/c3-project
./run_carla.sh
// Create new tab
cd /home/workspace/c3-project
./cloud_loc // Might have core dump on start up, just rerun if so. Crash doesn't happen more than a couple of times
```


### Result
As seen on the video, pose error is too much for localization. This error might be decreased by tuning ICP parameters. Also, it is depend on performance of the workspace computer.

https://user-images.githubusercontent.com/60349121/182038806-6d994a10-e3a4-460f-b815-e4a3acdce974.mp4

