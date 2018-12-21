## Intro

It is a simple Multiple Object tracker(MOT) by Detection results. 

__Detection__

There are a few detection algorithm such as ACF, DPM, HOG, ICF, etc. All of them are a part of : https://bitbucket.org/fdesmedt/openframework/overview

__Multiple Tracker__

Tracker is based on Kalman Filter and Data Association. For Data association, Hungarian Algorithm is used. 

you can see results in "https://youtu.be/H0lKRKsV5dc"

## Dependency
OpenCV is required.
Tracker don't need any libraries, but to use Detector you should get below libraries.
1. Eigen ( you can get in, http://eigen.tuxfamily.org/index.php?title=Main_Page)
2. Boost ( you can get in, https://www.boost.org/users/download/#live)
3. RapidXml ( you can get in, https://sourceforge.net/projects/rapidxml/)

## Usage
Confirm parameters of MOT. There are two parameters to control occlusion, detection miss, or tracking miss in config.h
```
const int   MAX_MISS_FRAME = 2;
const float MAX_DISTANCE = 100.0;
```
If you want to change Detector to otheres, change below paramater
```
// ACF 0
// DPM 1
// ICF 2
// HOG 3
DetectorManager DM(0);
``` 
