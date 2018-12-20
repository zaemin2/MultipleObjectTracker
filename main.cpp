//
//  main.cpp
//  MOTracker
//
//  Created by Jaemin LEE on 2018. 12. 4.
//  Copyright © 2018년 Jaemin LEE. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Detector/Core/DetectionList.h"
#include "Detector/Core/NonMaximumSuppression.h"
#include "Detector/Core/DetectorManager.h"
#include "MOT/WMultiTracker.hpp"


int main(int argc, char* argv[]) {
    cv::VideoCapture cap;
    std::string fileName;
    bool playVideo = true;
    
    // start frame number
    int n_frame = 0;
    
    if (argc > 1) {
        if (cap.open(argv[1])) {
            std::cout << "Openning " << fileName << std::endl;
        }
        else {
            std::cout << "FILE OPEN ERROR " << std::endl;
            return -1;
        }
    }
    else {
        std::cout << "NOT found video " << argv[1] << std::endl;
        return 0;
    }
    
    double fps=0;
    int64 t1,t0 = cvGetTickCount(); int fnum=0;
    char sss[256];
    std::string text;
    
    
    cv::Mat frame;
    
    // checking time
    clock_t begint, endt;
    
    cap.set(CV_CAP_PROP_POS_FRAMES, n_frame);
    cap >> frame;
    
    DetectorManager DM(0);
    NonMaximumSuppression NMS;
    WMultiTracker mot;
    
    float detect_time, track_time;
    
    while (1) {
        if (playVideo) {
            
            if (!cap.read(frame)) {
                break;
            }
            
            cv::Mat canvas = frame.clone();
            std::vector<cv::Rect2d> detections;
            
            begint = clock();
            // Apply detector
            DetectionList DL = DM.applyDetector(frame);
            // NMS
            DetectionList NMax = NMS.dollarNMS(DL);
            NMax.getDetections(detections);
            endt = clock();
            detect_time = endt - begint;
            
            // Tracking
            begint = clock();
            mot.run(frame, detections);
            endt = clock();
            track_time = endt - begint;
            
            //NMax.Draw(canvas);
            mot.draw(canvas);
            
            
            //draw framerate on display image
            if(fnum >= 9){
                t1 = cvGetTickCount();
                fps = 10.0/((double(t1-t0)/cvGetTickFrequency())/1e+6);
                t0 = t1; fnum = 0;
            }else fnum += 1;

            sprintf(sss,"%d frames/sec",(int)round(fps)); text = sss;
            cv::putText(canvas,text,cv::Point(10,20),
                            CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
            
            
            cv::imshow("Canvas", canvas);
            std::cout << "Detect :" << detect_time << ", Track:" << track_time << std::endl;
            
            n_frame++;
        }
        
        
        char key = cv::waitKey(1);
        if (key == 'p') {
            playVideo = !playVideo;
        }
        if (key == 's') {
            cap.set(CV_CAP_PROP_POS_FRAMES, n_frame -=20);
        }
        if (key == 'd') {
            cap.set(CV_CAP_PROP_POS_FRAMES, n_frame += 20);
        }
    }
    return 0;
}
