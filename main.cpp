#include <iostream>
#include <unistd.h>

#include "vision/vision.h"

#define maxdist 4.0f
#define distpixel 0.006f
#define matlenpx 800

#define hrange 2.0f

// ============================================================================
int main(int argc, char **argv) {
    bool QuitProgram = false;
    cv::Mat processed_frame;
    std::vector<vision::T3Dpoint> pc;
    vision::T3Dpoint pt;
    cv::Mat ptshow(matlenpx,matlenpx,CV_8UC3,cv::Scalar(0,127,127));
    int u,v;
    cv::Vec3b px;
    
    std::vector<vision::TScanPoint> sc;
    vision::TScanPoint sp;
    
    bool ShowPoints = true;
    
    std::cout << "Saya the robot Test bench" << std::endl;
    
// ----------------------------------------------------------------------------
    vision::StartPcRow = 160;
    vision::EndPcRow = D455H-60; //vision::EndPcRow = D455H-160;
    vision::Init();
    
    while (!QuitProgram) {
        vision::Frame();
        if (vision::NewD455) {
            cv::imshow("RGB",vision::RGB_image);
            
            vision::depth_image16.convertTo(processed_frame,CV_8U,255.0 / D455depth_color_max,0.0);
            cv::applyColorMap(processed_frame,processed_frame,cv::COLORMAP_JET);
            
            cv::line(processed_frame,cv::Point(vision::IgnoreFromLeft,0),cv::Point(vision::IgnoreFromLeft,D455H-1),{255,255,255},1);
            cv::line(processed_frame,cv::Point(0,vision::StartPcRow),cv::Point(D455W-1,vision::StartPcRow),{255,255,255},1);
            cv::line(processed_frame,cv::Point(0,vision::EndPcRow-1),cv::Point(D455W-1,vision::EndPcRow-1),{255,255,255},1);
            
            cv::imshow("Depth",processed_frame);
            //std::cout << vision::PointCloud.size() << std::endl;
            pc = vision::PointCloud; // deep copy
            //std::cout << vision::ScPoints.size() << std::endl;
            sc = vision::ScPoints;
            vision::NewD455 = false;
        }

        ptshow.setTo(cv::Scalar(0,0,0));
        if (ShowPoints) {
            for (int i=0;i<pc.size();i++) {
                pt = pc[i];
                u = static_cast<int>(pt.x / distpixel)+(matlenpx/2);
                if (u < 0) u = 0;
                if (u > matlenpx-1) u = matlenpx-1;
                v = static_cast<int>(pt.z / distpixel)+(matlenpx/2);
                if (v < 0) v = 0;
                if (v > matlenpx-1) v = matlenpx-1;
                
                float h = pt.y;
                h = --h;
                if (h > hrange) h = hrange;
                if (h < -hrange) h = -hrange;
                float sc = hrange/255;            
                uint8_t G = static_cast<uint8_t>(h/sc);
                ptshow.at<cv::Vec3b>(matlenpx-v,u) = {G,G,G};
            }
        }
                
        for (int i=0;i<sc.size();i++) {
            sp = sc[i];
            u = static_cast<int>(sp.x / distpixel)+(matlenpx/2);
            if (u < 0) u = 0;
            if (u > matlenpx-1) u = matlenpx-1;
            v = static_cast<int>(sp.z / distpixel)+(matlenpx/2);
            if (v < 0) v = 0;
            if (v > matlenpx-1) v = matlenpx-1;
            ptshow.at<cv::Vec3b>(matlenpx-v,u) = {0,100,255};
        }
        
        ptshow.at<cv::Vec3b>(matlenpx/2,matlenpx/2) = {255,180,0};
        
        cv::imshow("Scan",ptshow);
        
        
        int key = cv::waitKey(1); // Wait for a key press for 1ms
        if (key == 27) { // ESC key
            QuitProgram = true;
        }
        else if (key == 'p') {
            ShowPoints = !ShowPoints;
        }
        else if (key == 's') {
            vision::StartPcRow--;
        }
        else if (key == 'S') {
            vision::StartPcRow++;
        }
        else if (key == 'e') {
            vision::EndPcRow--;
        }
        else if (key == 'E') {
            vision::EndPcRow++;
        }
        
        usleep(100000);
    }    
    return 0;
}
