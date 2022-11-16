#include "VergenceControl.h"
#include <bits/stdc++.h>

//g++ -std=c++17 main.cpp VergenceControl.cpp ./ini_lib/INIReader.cpp ./ini_lib/ini.cpp -o vergence  -lYARP_sig  `pkg-config --cflags --libs opencv`


main(int argc, char * argv[]){
    std::unordered_map<float, int> hm;

    vergencecontrol::VergenceControl vc(160,120, "../../iCub_Vergence/data/Gt43B0.0208f0.063ph7.ini");

    int cols, rows, counter =0;
    float * LImageData, * RImageData;
    float V1comp_resp[7][8][120][160];
    std::vector<cv::Mat>images;

    cv::VideoCapture LEye(argv[1]);
    cv::VideoCapture REye(argv[2]);

    if(!LEye.isOpened() || !REye.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while(1){
        if(counter ==2){
            break;
        }
        int WIDTH = 160, HEIGHT = 120;
        cv::Mat LFrame, RFrame;
        LEye>>LFrame;
        REye>>RFrame;
        cv::cvtColor(LFrame, LFrame, CV_BGR2GRAY);
        cv::cvtColor(RFrame, RFrame, CV_BGR2GRAY);

        cv::resize(LFrame, LFrame, cv::Size(WIDTH, HEIGHT), CV_INTER_LINEAR);
        cv::resize(RFrame, RFrame, cv::Size(WIDTH, HEIGHT), CV_INTER_LINEAR);
     
        if((LFrame.rows != RFrame.rows) || (RFrame.cols != LFrame.cols)){
            std::cout << " rows -> " << LFrame.rows << " " << RFrame.rows << " cols -> " << RFrame.cols << " " << LFrame.cols << " couter -> " << counter <<std::endl;
            std::cerr<<" the amount of rows or cols ist not identical"<<std::endl;
        }
        
        //rows = LFrame.rows;
        //cols = LFrame.cols;
        //RImageData = (float *) RFrame.data;
        //LImageData = (float *) LFrame.data;

        cv::imshow(" Left ", LFrame);
        cv::imshow(" Right ", RFrame);
        cv::waitKey(0);

        //vc.loadImgArr(rows, cols, LImageData, 'l');
        //vc.loadImgArr(rows, cols, RImageData, 'r');

        vc.loadMatImg(LFrame, 'l');
        vc.loadMatImg(RFrame, 'r');

        vc.computeFilterPatch();

        counter++;

        //vc.getV1compResponse_1_2_(V1comp_resp);
        vc.showV1compResponse_1_2(); 

        char c=(char)cv::waitKey(25);
        if(c==27)
            break; 

    }

    LEye.release();
    REye.release();

 
    // Closes all the frames
    cv::destroyAllWindows();


    return 0;

}
