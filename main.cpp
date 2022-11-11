#include "VergenceControl.h"
#include <bits/stdc++.h>
//g++ -std=c++17 main.cpp VergenceControl.cpp ./ini_lib/INIReader.cpp ./ini_lib/ini.cpp -o vergence  -lYARP_sig  `pkg-config --cflags --libs opencv`
typedef unsigned char uchar;
void transpose(float imageArr[7][8][120][160], std::vector<cv::Mat>&images ){
    //float img[160][120];
    for (int i = 0; i < 7; i++){
        for (int j = 0; j < 8; j++){
            //std::cout << sizeof(imageArr[i][j]) << std::endl;
            cv::Mat image = cv::Mat(120, 160, CV_32FC1, imageArr[i][j]) /255.0;
            //std::memcpy(image.data, imageArr[i][j], 120*160*sizeof(float));
            //images.push_back(image);
            /* for(int k = 0; k<120; k++){
                for(int l = 0; l<160; l++){
                    cv::Vec3b pix = image.at<float>(k,l);
                    std::cout<< (float)pix[0] << std::endl;
                }
            }     */
            cv::imshow(" disparity maps ", image);
            char c=(char)cv::waitKey(25);
            if(c==27)
                break;
                
        }
          
    }            
}


 
void countFreq(float arr[][160], std::unordered_map<float, int>&hm)
{
    // Insert elements and their
    // frequencies in hash map.
    for (int i=0; i<120; i++){
        for(int j =0; j<160; j++){
            hm[arr[i][j]]++;
        }
    }
        
}

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
        if(counter ==1){
            break;
        }
        int WIDTH = 200, HEIGHT = 300;
        cv::Mat LFrame, RFrame;
        LEye>>LFrame;
        REye>>RFrame;
        if((LFrame.rows != RFrame.rows) || (RFrame.cols != LFrame.cols)){
            std::cout << " rows -> " << LFrame.rows << " " << RFrame.rows << " cols -> " << RFrame.cols << " " << LFrame.cols << " couter -> " << counter <<std::endl;
            std::cerr<<" the amount of rows or cols ist not identical"<<std::endl;
        }
        rows = LFrame.rows;
        cols = LFrame.cols;
        RImageData = (float *) RFrame.data;
        LImageData = (float *) LFrame.data;
        //std::cout << " rows -> " << rows << " cols -> " << cols<< " couter -> " << counter <<std::endl;

        vc.loadImgArr(rows, cols, LImageData, 'l');
        vc.loadImgArr(rows, cols, RImageData, 'r');
        vc.computeFilterPatch();

     /*    cv::resizeWindow("Display", WIDTH, HEIGHT);
        cv::imshow("left", LFrame);
        cv::imshow("right", RFrame); */
        counter++;

        //vc.getV1compResponse_1_2_(V1comp_resp);
        vc.showV1compResponse_1_2(); 

        /* for(int i =0;i<7;i++){
            for (int j =0; j < 8; j++){
                countFreq(V1comp_resp[i][j], hm);
                std::cout << " size -> " << hm.size() << std::endl;
                for (auto const& x: hm){
                    std::cout << " key -> " << x.first << " value -> " << x.second<< std::endl;
                }  

                cv::Mat image = cv::Mat(120, 160, CV_32F, V1comp_resp[i][j]);
                //image.convertTo(image, CV_8UC1);
                cv::imwrite("disparity map", image);
                char c=(char)cv::waitKey(25);
                if(c==27)
                    break;  
            }
        } */
         
        //transpose(V1comp_resp, images);
       /*  char c=(char)cv::waitKey(25);
        if(c==27)
            break; */

    }

    LEye.release();
    REye.release();

 
    // Closes all the frames
    cv::destroyAllWindows();


    return 0;

}
