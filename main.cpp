#include <iostream>
#include <unistd.h>
#include <thread>

#include "vision/vision.h"
#include "LocalMap/locmap.h"
#include "t265.h"

#define DispVision
#define PlanIt

#define maxdist 4.0f
#define distpixel 0.006f
#define matlenpx 800

#define hrange 2.0f

#define GridUpscale 4.0

// ============================================================================
int main(int argc, char **argv) {
    bool QuitProgram = false;
    cv::Mat processed_frame;
    std::vector<T3Dpoint> pc;
    T3Dpoint pt;
    cv::Mat ptshow(matlenpx,matlenpx,CV_8UC3,cv::Scalar(0,127,127));
    int u,v;
    cv::Vec3b px;

    std::vector<TScanPoint> sc;
    TScanPoint sp;

    bool ShowPoints = true;

    std::thread LocMapThread;
    cv::Mat show_grid(GridCells,GridCells,CV_8UC3,cv::Scalar(127,127,127));
    cv::Mat up_grid;

    std::thread T265Thread;
    std::unique_lock<std::mutex> mainT265_lock(T265_mutex, std::defer_lock);

    //glm::vec3 OldBotPos;
    float yaw_rad,R;
    std::vector<float> ranges;

#define VFHheight 150
    #define VFHline 3
    //cv::Mat dispVFH(Sectors,VFHheight,CV_8UC3,cv::Scalar(127,127,127));
    cv::Mat dispVFH(VFHheight,3*Sectors,CV_8UC3,cv::Scalar(127,127,127));
    //float ScaleVFH = ((float) VFHheight) / std::exp(GridSizeM / 2.0);
    //float Hmax = (GridSizeM / 2.0);
    //float ScaleVFH = ((float) VFHheight) / (Hmax * Hmax * Hmax);
    //float ScaleVFH = ((float) VFHheight) / (Hmax);
    float ScaleVFH = ((float) VFHheight) / Dmax;
    int H;

// ----------------------------------------------------------------------------
    std::cout << "Saya the robot Test bench" << std::endl;
    //vision::GetSerNo();
    //return 0;

// ----------------------------------------------------------------------------
    vision::StartPcRow = 160;
    vision::EndPcRow = D455H-60; //vision::EndPcRow = D455H-160;
    vision::Init();
    std::cout << "Vision running." << std::endl;

    locmap::ObstacleDelta = 0.15;
    locmap::InflateRadius = 3;
    LocMapThread = std::thread(&locmap::RunLocMap);

    t265::t265_serial_number = vision::t265_serial_number; // vision::GetSerNo(); aktualizuje i T265 serial number
    T265Thread = std::thread(&t265::RunT265);

    while (!QuitProgram) {
        vision::Frame();
        if (vision::NewD455) {
            //cv::imshow("RGB",vision::RGB_image);
#ifdef DispVision
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
#endif
            if (mainT265_lock.try_lock()) {
//                 Yaw = rs_yaw;
//                 YawRad = rs_yaw_rad;
//                 Pitch = rs_pitch;
//                 Roll = rs_roll;
//                 Velocity = rs_velocity;
//                 AngularVelocity = rs_angular_velo;
//                 PosX = rs_x;
//                 PosY = rs_y;
//                 PosZ = rs_z;

                yaw_rad = t265::rs_yaw_rad;
                BotPos = t265::rs_BotPos;
                BotOrientation = t265::rs_BotOrientation;
                mainT265_lock.unlock();
            }

            if (!locmap::UpdateGridMap) {
#ifdef PlanIt
                //locmap::SetGoal(0,GridCenter);
                locmap::SetGoal(0,GridCells-1);
                locmap::Plan(); // TODO Force A* planner to plan to the nearest wall even if goal is unreachable
#endif
                show_grid.setTo(cv::Scalar(127,127,127));
                for (int y = 0; y < GridCells; ++y) {
                    for (int x = 0; x < GridCells; ++x) {
                        if (locmap::ObstacleGrid[x][y].free) {
                            show_grid.at<cv::Vec3b>(x,y) = {255,255,255};
                        }
                        if (locmap::ObstacleGrid[x][y].obstacle) {
                            show_grid.at<cv::Vec3b>(x,y) = {160,160,80};
                        }
                        if (locmap::ObstacleGrid[x][y].occupied) {
                            show_grid.at<cv::Vec3b>(x,y) = {0,0,0};
                        }
//                         if (locmap::ObstacleGrid[x][y].path) {
//                             show_grid.at<cv::Vec3b>(x,y) = {180,0,255};
//                         }
                    }
                }

                //int startX,startY,endX,endY;
                int X,Y;
                TPolarVector pV;
                TPoint2D P;
                std::vector<TPoint2D> pts;

                //cv::putText(show_grid,"Hello, OpenCV!",cv::Point(10, show_grid.rows / 2),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(118, 185, 0),1);
                pV.X = GridCenter;
                pV.Y = GridCenter;
                pV.Mag = GridCells/4.0;
                pV.Theta = yaw_rad - M_PI/2; // M_PI/2 je kvuli zobrazeni 0 rad na "sever" mapy. Jinak chodi "matematicky" 0 = vpravo a roste ccw
                PolarToCart(&pV);
                X = round(pV.X); Y = round(pV.Y);
                X += GridCenter; Y += GridCenter;
                //Bresenham(GridCenter,GridCenter,X,Y,pts);
                //BresenhamLim(GridCenter,GridCenter,X,Y,0,GridCells-1,pts);
                BresenhamLim2(GridCenter,GridCenter,X,Y,0,GridCells-1,pts);

                for (int i=0; i< pts.size(); i++) {
                    show_grid.at<cv::Vec3b>(pts[i].y,pts[i].x) = {0,255,255};
                }
                show_grid.at<cv::Vec3b>(GridCenter,GridCenter) = {255,0,0};

// VFH
//#if 0
                locmap::CalcVFH();
                int D=0;
                dispVFH.setTo(cv::Scalar(255,255,255));
#if 1
                for (int i=0; i<Sectors; i++) {
                    R = locmap::VFHisto[i]; // Rozsah <0,1>, kde 1 = nejblize
                    H = (VFHheight-1) - round(R * ScaleVFH);
                    cv::line(dispVFH,cv::Point(VFHline*i,VFHheight-1),cv::Point(VFHline*i,H),{0,0,0},VFHline);


#if 0
//                     R = locmap::VFHisto[i];
//                     R = R * ScaleVFH;
//                     H = 0;
                    R = locmap::RawHisto[i];
                    //R = Hmax - locmap::VFHisto[i];
                    if (R > 0.0) {
                        H = (VFHheight-1) - round(R * ScaleVFH);
                        //H = (VFHheight-1) - round(R);
                        //H = round(R * ScaleVFH);
                    }
                    else {
                        H = VFHheight-1;
                        //H = 0;
                    }
                    //H = (VFHheight-1) - round(R * ScaleVFH);
                    cv::line(dispVFH,cv::Point(VFHline*i,VFHheight-1),cv::Point(VFHline*i,H),{0,0,0},VFHline);

                    //cv::line(showDist,cv::Point(i,ScanHt-1),cv::Point(i,DistY),{0,0,0});
                    //cv::line(dispVFH,cv::Point(VFHline*i,VFHheight-1),cv::Point(VFHline*i,H),{0,0,0},VFHline);
                    //cv::line(dispVFH,cv::Point(VFHline*i,0),cv::Point(VFHline*i,D),{0,0,0},VFHline);


//                     cv::line(dispVFH,cv::Point(VFHline*i,VFHheight-1),cv::Point(VFHline*i,VFHheight-1-D),{0,0,0},VFHline);
//                     D++;
//                     if (D > VFHheight-1) D = 0; //VFHheight-1;
#endif
                }
#endif
                D = Sectors / 2;
                cv::line(dispVFH,cv::Point(VFHline*D,VFHheight-1),cv::Point(VFHline*D,0),{0,255,0},VFHline);
                D = Sectors / 4;
                cv::line(dispVFH,cv::Point(VFHline*D,VFHheight-1),cv::Point(VFHline*D,0),{255,255,0},VFHline);
                D = 3* (Sectors / 4);
                cv::line(dispVFH,cv::Point(VFHline*D,VFHheight-1),cv::Point(VFHline*D,0),{0,128,255},VFHline);
                cv::imshow("VFH",dispVFH);

                locmap::lmap_depth_image = vision::depth_image16.clone();
                locmap::UpdateGridMap = true;
            }

            vision::NewD455 = false;
        }

        cv::resize(show_grid,up_grid,cv::Size(),GridUpscale,GridUpscale,cv::INTER_NEAREST); // resize bez interpolace
        cv::imshow("Map",up_grid);

//         if ((OldBotPos.x != BotPos.x) || (OldBotPos.y != BotPos.y) || (OldBotPos.z != BotPos.z)) {
//             //std::cout << "." << std::endl;
//             //std::cout << "." << std::flush;
//             std::cout << "x: " << BotPos.x << " y: " << BotPos.y << " z: " << BotPos.z << std::endl;
//         }
//         OldBotPos = BotPos;


#ifdef DispVision
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
#endif

        int key = cv::waitKey(1); // Wait for a key press for 1ms
        if (key == 27) { // ESC key
            QuitProgram = true;
        }
        else if (key == 'c') {
            locmap::ClearLocMap();
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

        //usleep(100000);
        usleep(10000);
    }

//     for (int i=0; i < Sectors; i++) {
//         std::cout << locmap::RawHisto[i] << std::endl;
//         //std::cout << locmap::VFHisto[i] << std::endl;
//     }

    std::cout << "down T265" << std::endl;
    t265::ShutdownT265 = true;
    T265Thread.join();

    std::cout << "down LocalMap" << std::endl;
    locmap::ShutdownLocMap = true;
    LocMapThread.join();

    return 0;
}
