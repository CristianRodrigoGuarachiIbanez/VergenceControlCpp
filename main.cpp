#include "VergenceControl.h"
#include <bits/stdc++.h>

//g++ -std=c++17 main.cpp VergenceControl.cpp ./ini_lib/INIReader.cpp ./ini_lib/ini.cpp -o vergence  -lYARP_sig  `pkg-config --cflags --libs opencv`

inline void transpose4D(float input[320][240][7][8], float output[7][8][320][240]){

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 320; k++)
                for (int l = 0; l < 240; l++) {
                    output[i][j][k][l] = input[k][l][i][j];
                }
}
inline void transpose4D_2(float input[160][120][7][8], float output[7][8][160][120]){

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 160; k++)
                for (int l = 0; l < 120; l++) {
                    output[i][j][k][l] = input[k][l][i][j];
                    std::cout<< " " << input[k][l][i][j] << std::endl;
                }
}

inline void showImages(std::vector<cv::Mat>&images){
    for (int i =0; i<images.size(); i++){
        cv::imshow("images", images[i]);
        cv::waitKey(0);
    }
}

inline void writeImages(std::vector<cv::Mat>&images){
    for (int i =0; i<images.size(); i++){
        cv::imwrite("./images/dp_" + std::to_string(i) + ".png", images[i]);
    }
}


main(int argc, char *argv[]){

    
    int WIDTH, HEIGHT;
    std::string argv3 = std::string(argv[3]);

    if(argv3=="array1" || argv3=="array2"|| argv3=="show_1_2"){
        WIDTH = 160;
        HEIGHT = 120;
        
    }else if(argv3 == "resp1" || argv3=="resp2" || argv3=="show_full"){
        WIDTH = 320;
        HEIGHT = 240;
    }
    
    vergencecontrol::VergenceControl vc(160,120, "../data/Gt43B0.0208f0.063ph7.ini");

    int cols, rows, counter =0;
    float * LImageData, * RImageData;

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

        cv::imshow(" Left ", LFrame);
        cv::imshow(" Right ", RFrame);
        cv::waitKey(0);

        rows = LFrame.rows;
        cols = LFrame.cols;

        if(argv3=="array1" || argv3=="array2" || argv3=="show_1_2" ){
            
            //RImageData = (float *) RFrame.data;
            //LImageData = (float *) LFrame.data;

            //vc.loadImgArr(rows, cols, LImageData, 'l');
            //vc.loadImgArr(rows, cols, RImageData, 'r');
            vc.loadMatImg(LFrame, 'l');
            vc.loadMatImg(RFrame, 'r');
            vc.computeFilterPatch();
            
            if(argv3=="array1"){
                float V1comp_resp[160][120][7][8];
                vc.getV1compResponse_1_2(V1comp_resp);
            }else if(argv3=="array2"){
                std::vector<cv::Mat> V1comp_resp;
                V1comp_resp.reserve(56);
                vc.getV1compResponse_1_2_(V1comp_resp);
                //showImages(V1comp_resp);
                writeImages(V1comp_resp);
            }else if(argv3 == "show_1_2"){
                vc.showV1compResponse_1_2(); 
            }
        
        }else if (argv3=="resp1" || argv3=="resp2" ||  argv3 == "show_full"){
            vc.loadMatImg(LFrame, 'l');
            vc.loadMatImg(RFrame, 'r');

            vc.computeFilterPatch();

            if(argv3=="resp1"){
                //float V1comp_resp[320][240][7][8];
                //float images[7][8][240][320];
                //vc.getV1compResponse_full(V1comp_resp);
            }else if(argv3=="resp2"){
                 std::vector<cv::Mat> V1comp_resp;
                 V1comp_resp.reserve(56);
                vc.getV1compResponse_full_(V1comp_resp);
                //showImages(V1comp_resp);
                writeImages(V1comp_resp);
            }else if(argv3 == "show_full"){
                vc.showV1compResponse_full();
            }
            
        }
       
                
        
        counter++;
    }

    LEye.release();
    REye.release();

 
    // Closes all the frames
    cv::destroyAllWindows();


    return 0;

}
