//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%              A Portable Bio-Inspired Architecture for              %%%%
//%%%%                  Efficient Robotic Vergence Control                %%%%
//%%%%                                                                    %%%%
//%%%%                                                                    %%%%
//%%%%  Copyright (c) Sep. 2017                                           %%%%
//%%%%  All rights reserved.                                              %%%%
//%%%%                                                                    %%%%
//%%%%  Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini     %%%%
//%%%%                                                                    %%%%
//%%%%  PSPC-lab - Department of Informatics, Bioengineering,             %%%%
//%%%%  Robotics and Systems Engineering - University of Genoa            %%%%
//%%%%                                                                    %%%%
//%%%%  The code is released for free use for SCIENTIFIC RESEARCH ONLY.   %%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// header include
#include "VergenceControl.h"

// yarp includes
#include <yarp/cv/Cv.h>
#include <yarp/sig/all.h>

// system includes
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// INI-Reader includes
#include "ini_lib/INIReader.h"

namespace vc = vergencecontrol;
namespace y_cv = yarp::cv;
namespace y_sig = yarp::sig;

vc::VergenceControl::VergenceControl(int width, int height, const std::string &filter_filename) {
    //------------------- DEFAULT CONSTRUCTOR -------------------//
    M = height;
    N = width;

    // INIT RGB IMAGE TO 0
    L = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    R = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    //------------------- CREATE GABOR FILTERS AND GAUSSIAN ENVELOPE
    //-------------------//
    loadGaborParams(filter_filename);

    Mpatch = M;
    Npatch = N;

    // Mcut = Gfilt.taps;
    // Ncut = Gfilt.taps;

    // Mpatch = 2 * Gfilt.taps;
    // Npatch = 2 * Gfilt.taps;

    // Mpad = Mcut + 2 * Gfilt.taps;
    // Npad = Mcut + 2 * Gfilt.taps;

    createGaborFilters();
    create_Gaussian(filter_filename);

    //------------------- INIT PATCH IMAGE -------------------//
    Lpatch = cv::Mat(Mpatch, Npatch, CV_8UC1, cv::Scalar(0));
    Rpatch = cv::Mat(Mpatch, Npatch, CV_8UC1, cv::Scalar(0));

    Lpatch32F = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0));
    Rpatch32F = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0));

    //------------------- INIT PADDED IMAGE -------------------//
    // Lpad = cv::Mat(height + 2 * Gfilt.taps, width + 2 * Gfilt.taps, CV_8UC3, cv::Scalar(0, 0, 0));
    // Rpad = cv::Mat(height + 2 * Gfilt.taps, width + 2 * Gfilt.taps, CV_8UC3, cv::Scalar(0, 0, 0));

    //------------------- SET ROI TO IMAGE CENTER -------------------//
    cv::Point2d C(N / 2, M / 2);
    setCenter(C.x, C.y);

    //------------------- INIT FILTERED IMAGE -------------------//
    Lfilt[0] = new cv::Mat1f[Gfilt.Nori];    // Even Filters
    Lfilt[1] = new cv::Mat1f[Gfilt.Nori];    // Odd Filters
    Rfilt[0] = new cv::Mat1f[Gfilt.Nori];    // Even  Filters
    Rfilt[1] = new cv::Mat1f[Gfilt.Nori];    // Odd  Filters
    for (int P = 0; P < 2; P++)              // EVEN/ODD
        for (int T = 0; T < Gfilt.Nori; T++) {
            Lfilt[P][T] = cv::Mat(Mpatch, Npatch, CV_32FC1);
            Rfilt[P][T] = cv::Mat(Mpatch, Npatch, CV_32FC1);
        }

    //------------------- INIT FILTERED AND PHASE SHIFTED IMAGE + ENERGY MATRIX
    //-------------------//
    for (int P = 0; P < Gfilt.Nph; P++) {          // phases
        RShiftE[P] = new cv::Mat1f[Gfilt.Nori];    // Even  Filters
        RShiftO[P] = new cv::Mat1f[Gfilt.Nori];    // Odd  Filters
        Energy[P] = new cv::Mat1f[Gfilt.Nori];     // Binocular Energy

        for (int T = 0; T < Gfilt.Nori; T++) {
            RShiftE[P][T] = cv::Mat(Mpatch, Npatch, CV_32FC1);
            RShiftO[P][T] = cv::Mat(Mpatch, Npatch, CV_32FC1);
            Energy[P][T] = cv::Mat(Mpatch, Npatch, CV_32FC1);
        }
    }

    SUM_H = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));
    SUM_V = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));
    SUM_N = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));

    planes[0] = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));
    planes[1] = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));

    tmpEven = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));
    tmpOdd = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));
    tmpSum = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));

    tmpF = cv::Mat(Mpatch, Npatch, CV_32FC1, cv::Scalar(0.0));

    //------------------- PRINT INFO -------------------//
    // printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    //        "%%%%%%%%%%%%%%%%%%%%%%%\n");
    // printf("%%%%    A Portable Bio-Inspired Architecture for %%%%\n");
    // printf("%%%%       Efficient Robotic Vergence Control %%%%\n");
    // printf("%%%% %%%%\n"); printf("%%%% %%%%\n"); printf("%%%%    Copyright (c)
    // Sep. 2017                                            %%%%\n"); printf("%%%%
    // All rights reserved. %%%%\n"); printf("%%%% %%%%\n"); printf("%%%% Authors:
    // Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini      %%%%\n");
    // printf("%%%% %%%%\n"); printf("%%%%    PSPC-lab - Department of
    // Informatics, Bioengineering,              %%%%\n"); printf("%%%% Robotics
    // and Systems Engineering - University of Genoa             %%%%\n");
    // printf("%%%% %%%%\n"); printf("%%%%    The code is released for free use
    // for SCIENTIFIC RESEARCH ONLY.    %%%%\n");
    // printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    //        "%%%%%%%%%%%%%%%%%%%%%%%\n");
    printf("\n\nImages: width=%d, height=%d;\n\n", N, M);
    printf("Gabor Filters: orientations=%d; phases=%d; \n", Gfilt.Nori, Gfilt.Nph);
    printf("               fo=%f; sigma=%f; tapsX=%d; tapsY=%d\n", Gfilt.f, Gfilt.sigma, Gfilt.taps, Gfilt.taps);
    printf("\n");
}

vc::VergenceControl::~VergenceControl() {
    // -------------------DISTRUCTOR------------------- //
    L.release();
    R.release();
    Lpatch.release();
    Rpatch.release();
}

void vc::VergenceControl::loadGaborParams(const std::string &ini_filter_file) {
    //------------------- LOAD PARAMETERS FOR GABOR FILTERS -------------------//

    // LOAD FILTER PARAMS

    INIReader iniReader(ini_filter_file);
    Gfilt.Nori = iniReader.GetInteger("Gabor", "Nori", 0);
    Gfilt.Nph = iniReader.GetInteger("Gabor", "Nph", 0);
    Gfilt.sigma = static_cast<float>(iniReader.GetReal("Gabor", "sigma", 0.0));
    Gfilt.taps = iniReader.GetInteger("Gabor", "taps", 0);

    Gfilt.B = static_cast<float>(iniReader.GetReal("Gabor", "B", 0.0));
    Gfilt.f = static_cast<float>(iniReader.GetReal("Gabor", "f", 0.0));

    std::string phase = iniReader.Get("Gabor", "phases", "0.0 0.0 0.0 0.0 0.0 0.0 0.0");
    if (phase[0] == '\"') {
        phase.erase(0, 1);
    }
    if (phase[phase.size() - 1] == '\"') {
        phase.erase(phase.size() - 1, 1);
    }

    std::vector<float> v;
    for (std::string::size_type sz = 0; sz < phase.size();) {
        phase = phase.substr(sz);
        v.push_back(stof(phase, &sz));
    }
    Gfilt.phase = new float[v.size()];
    for (unsigned int i = 0; i < v.size(); i++) {
        Gfilt.phase[i] = v[i];
    }

    fovea_size = static_cast<float>(iniReader.GetReal("Gaussian", "fovea_size", 0.0));
}

void vc::VergenceControl::createGaborFilters() {
    //------------------- CREATE GABOR FILTERS -------------------//
    // Gfilt.taps = 2 * Gfilt.taps;
    Gfilt.taps = Npatch;
    // Gfilt.taps = 80;

    Gfilt.HalfSize = (Gfilt.taps - Gfilt.taps % 2) / 2;

    Gfilt.theta = reinterpret_cast<float *>(malloc(sizeof(float) * Gfilt.Nori));
    for (int i = 0; i < Gfilt.Nori; i++) Gfilt.theta[i] = static_cast<float>(i * M_PI / Gfilt.Nori);
    Gfilt.phi[0] = static_cast<float>(M_PI / 2.0);
    Gfilt.phi[1] = 0.0;

    meshgridTest(cv::Range(-Gfilt.HalfSize, Gfilt.HalfSize - 1), cv::Range(-Gfilt.HalfSize, Gfilt.HalfSize - 1), Gfilt.X, Gfilt.Y);

    std::cout << Gfilt.X << std::endl;
    std::cout << Gfilt.Y << std::endl;

    Gfilt.Filters[0] = new cv::Mat1f[Gfilt.Nori];    // Even Filters
    Gfilt.Filters[1] = new cv::Mat1f[Gfilt.Nori];    // Odd  Filters

    for (int P = 0; P < 2; P++)    // EVEN/ODD
        for (int T = 0; T < Gfilt.Nori; T++) {
            Gfilt.Filters[P][T] = cv::Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);

            Gabor2D(Gfilt.X, Gfilt.Y, Gfilt.Filters[P][T], Gfilt.f, Gfilt.theta[T], Gfilt.sigma, Gfilt.phi[P]);
            Gfilt.Filters[P][T] = Gfilt.Filters[P][T] - mean(Gfilt.Filters[P][T]);
        }
}

void vc::VergenceControl::Gabor2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G, float f, float theta, float sigma, float phi) {
    //------------------- 2D GABOR FUNCTION -------------------//
    float Xrot, Yrot;

    for (int r = 0; r < X.rows; r++)
        for (int c = 0; c < X.cols; c++) {
            Xrot = X.at<int>(r, c) * cos(theta) + Y.at<int>(r, c) * sin(theta);
            Yrot = -X.at<int>(r, c) * sin(theta) + Y.at<int>(r, c) * cos(theta);

            G.at<float>(r, c) =
                static_cast<float>(exp(-(Xrot * Xrot + Yrot * Yrot) / (2 * sigma * sigma)) * sin(2 * M_PI * f * Xrot + phi));
        }
}

void vc::VergenceControl::create_Gaussian(const std::string &ini_filter_file) {
    //------------------- CREATE 2D GAUSSIAN FUNCTION -------------------//
    INIReader iniReader(ini_filter_file);
    fovea_size = static_cast<float>(iniReader.GetReal("Gaussian", "fovea_size", 0.0));

    Gaussian = cv::Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
    Gaussian2D(Gfilt.X, Gfilt.Y, Gaussian, fovea_size);
}

void vc::VergenceControl::Gaussian2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G, float sigma) {
    //------------------- 2D GAUSSIAN FORMULA -------------------//
    for (int r = 0; r < X.rows; r++)
        for (int c = 0; c < X.cols; c++)
            G.at<float>(r, c) = exp(-(X.at<int>(r, c) * X.at<int>(r, c) + Y.at<int>(r, c) * Y.at<int>(r, c)) / (2 * sigma * sigma));
}

void vc::VergenceControl::meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y) {
    //------------------- MESHGRID UTILITY -------------------//
    repeat(xgv.reshape(1, 1), static_cast<int>(ygv.total()), 1, X);
    repeat(ygv.reshape(1, 1).t(), 1, static_cast<int>(xgv.total()), Y);
}

// helper function (maybe that goes somehow easier)
void vc::VergenceControl::meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y) {
    //------------------- MESHGRID UTILITY -------------------//
    std::vector<int> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
    for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
    meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

void vc::VergenceControl::loadImgFile(const std::string &img_filename, char c) {
    //------------------- LOAD IMAGE INTO CLASS FROM FILE -------------------//
    if (c == 'L' || c == 'l') {
        //------------------- LOAD IMAGE -------------------//
        L = cv::imread(img_filename, cv::IMREAD_GRAYSCALE);

        if (L.empty()) {
            printf("Cannot read image file: %s\n", img_filename.c_str());
        }

        //------------------- PAD IMAGE -------------------//
        copyMakeBorder(L, Lpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, cv::BORDER_REFLECT_101);

        //------------------- PATCH IMAGE -------------------//
        Lpatch = Lpad(ROI);

        Lpatch.convertTo(Lpatch32F, CV_32FC1);
        Lpatch32F = Lpatch32F / 255.0;
        Lpatch32F = Lpatch32F - mean(Lpatch32F);

    } else if (c == 'R' || c == 'r') {
        //------------------- LOAD IMAGE -------------------//
        R = cv::imread(img_filename, cv::IMREAD_GRAYSCALE);

        if (R.empty()) {
            printf("Cannot read image file: %s\n", img_filename.c_str());
        }

        //------------------- PAD IMAGE -------------------//
        copyMakeBorder(R, Rpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, cv::BORDER_REFLECT_101);

        //------------------- PATCH IMAGE -------------------//
        Rpatch = Rpad(ROI);

        Rpatch.convertTo(Rpatch32F, CV_32FC1);
        Rpatch32F = Rpatch32F / 255.0;
        Rpatch32F = Rpatch32F - mean(Rpatch32F);
    }
}

void vc::VergenceControl::loadImg(y_sig::ImageOf<y_sig::PixelMono> &img, char c) {
    //------------------- LOAD IMAGE INTO CLASS -------------------//
    if (c == 'L' || c == 'l') {
        L = y_cv::toCvMat(img);
        // std::cout << "L: w: " << img.width() << " h: " << img.height() << std::endl;
        // std::cout << "L: w: " << L.size[0] << " h: " << L.size[1] << std::endl;

        // copyMakeBorder(L, Lpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, cv::BORDER_REFLECT_101);
        // Lpatch = Lpad(ROI);
        // Lpatch.convertTo(Lpatch32F, CV_32FC1);

        L.convertTo(Lpatch32F, CV_32FC1);
        Lpatch32F = Lpatch32F / 255.0;

    } else if (c == 'R' || c == 'r') {
        R = y_cv::toCvMat(img);
        // std::cout << "R: w: " << img.width() << " h: " << img.height() <<
        // std::endl;

        // copyMakeBorder(R, Rpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, cv::BORDER_REFLECT_101);
        // Rpatch = Rpad(ROI);
        // Rpatch.convertTo(Rpatch32F, CV_32FC1);

        R.convertTo(Rpatch32F, CV_32FC1);
        Rpatch32F = Rpatch32F / 255.0;
    }
}

void vc::VergenceControl::loadImgArr(int rows, int cols, float* img, char c) {
    //------------------- LOAD IMAGE INTO CLASS -------------------//
    if (c == 'L' || c == 'l') {
        Lpatch32F = cv::Mat(rows, cols, CV_32FC1, img);
        // std::cout << "L: w: " << img.width() << " h: " << img.height() << std::endl;
        // std::cout << "L: w: " << L.size[0] << " h: " << L.size[1] << std::endl;
        cv::imshow("left", Lpatch32F);
       

    } else if (c == 'R' || c == 'r') {
        Rpatch32F = cv::Mat(rows, cols, CV_32FC1, img);
        // std::cout << "R: w: " << img.width() << " h: " << img.height() <<
        // std::endl;
         cv::imshow("right", Rpatch32F);
    }
    cv::waitKey(0);
}

void vc::VergenceControl::loadMatImg(cv::Mat&img, char c){
    if(img.empty()){
        std::cerr << "Error\n";
        std::cerr << "Cannot Read Image in Constructor\n";
    }

    if (c == 'L' || c == 'l') {
        img.convertTo(Lpatch32F, CV_32FC1);
        Lpatch32F = Lpatch32F / 255.0;
        std::cout<< "cols -> " << Lpatch32F.cols << " rows -> " << Lpatch32F.rows << std::endl;
        //cv::imwrite("./images/input_left.png" , Lpatch32F);
        //cv::imshow("images/left", Lpatch32F);

    } else if (c == 'R' || c == 'r') {
        img.convertTo(Rpatch32F, CV_32FC1);
        Rpatch32F = Rpatch32F / 255.0;
        std::cout<< "cols -> " << Rpatch32F.cols << " rows -> " << Rpatch32F.rows << std::endl;
        //cv::imwrite("./images/input_right.png" , Rpatch32F);
        //cv::imshow("images/righ", Rpatch32F);
    }
    //cv::waitKey(0);
}

void vc::VergenceControl::computeFilterPatch() {
    //------------------- COMPUTE VERGENCE CONTROL -------------------//
    filtGaborBank();
    shiftGaborBank();
    computeEnergy();
}

void vc::VergenceControl::filtGaborBank() {
    //------------------- COMPUTE CONVOLUTION -------------------//
    for (int P = 0; P < 2; P++)                   // EVEN/ODD
        for (int T = 0; T < Gfilt.Nori; T++) {    // orientation
            filter2D(Lpatch32F, tmpF, 1, Gfilt.Filters[P][T]);
            tmpF.copyTo(Lfilt[P][T]);

            filter2D(Rpatch32F, tmpF, 1, Gfilt.Filters[P][T]);
            tmpF.copyTo(Rfilt[P][T]);
        }
}

void vc::VergenceControl::shiftGaborBank() {
    //------------------- SHIFT IN PHASE GABOR RESPONSE -------------------//
    for (int P = 0; P < static_cast<int>(Gfilt.Nph); P++)                                             // phases
        for (int T = 0; T < Gfilt.Nori; T++) {                                                        // orientation
            RShiftE[P][T] = cos(Gfilt.phase[P]) * Rfilt[0][T] - sin(Gfilt.phase[P]) * Rfilt[1][T];    // EVEN
            RShiftO[P][T] = cos(Gfilt.phase[P]) * Rfilt[1][T] + sin(Gfilt.phase[P]) * Rfilt[0][T];    // ODD
        }
}

void vc::VergenceControl::computeEnergy() {
    //------------------- COMPUTE BINOCULAR ENERGY -------------------//
    //std::cout << static_cast<int>(Gfilt.Nph)<<std::endl;
    for (int P = 0; P < static_cast<int>(Gfilt.Nph); P++)
        for (int T = 0; T < Gfilt.Nori; T++) {
            tmpEven = Lfilt[0][T] + RShiftE[P][T];
            tmpEven = tmpEven.mul(tmpEven);

            tmpOdd = Lfilt[1][T] + RShiftO[P][T];
            tmpOdd = tmpOdd.mul(tmpOdd);

            tmpSum = tmpEven + tmpOdd;

            sqrt(tmpSum, tmpSum);
            tmpSum.copyTo(Energy[P][T]);
        }
}

void vc::VergenceControl::setCenter(int x, int y) {
    //------------------- SET IMAGE LOCATION TO COMPUTE VERGENCE
    //-------------------//
    // SET IMAGE CENTER AND ROI
    Center.x = x + Gfilt.taps;
    Center.y = y + Gfilt.taps;
    ROI = cv::Rect(static_cast<int>(Center.x - Mpatch / 2), static_cast<int>(Center.y - Npatch / 2), Mpatch,
                   Npatch);    // X Y width height
}

void vc::VergenceControl::getCenter(int *x, int *y) {
    //------------------- GET SIZE OF IMAGE PATCH -------------------//
    x[0] = Center.x - Gfilt.taps;
    y[0] = Center.y - Gfilt.taps;
}

void vc::VergenceControl::getSubImgSize(int *M, int *N) {
    //------------------- GET SIZE OF IMAGE PATCH -------------------//
    M[0] = Mpatch;
    N[0] = Npatch;
}

void vc::VergenceControl::getV1compResponse_1_4(float V1comp_resp[80][60][7][8]) {
    //------------------- GET V1 complex Response -------------------//
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 80; k++)
                for (int l = 0; l < 60; l++) {
                    V1comp_resp[k][l][i][j] = Energy[i][j][l][k];
                }
}

void vc::VergenceControl::getV1compResponse_1_2(float V1comp_resp[160][120][7][8]) {
    //------------------- GET V1 complex Response -------------------//
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 160; k++)
                for (int l = 0; l < 120; l++) {
                    V1comp_resp[k][l][i][j] = Energy[i][j][l][k];
                }
}

void vc::VergenceControl::getV1compResponse_1_2_(float V1comp_resp[7][8][120][160]) {
    //------------------- GET V1 complex Response -------------------//
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 120; k++)
                for (int l = 0; l < 160; l++) {
                    V1comp_resp[i][j][k][l] = Energy[i][j][k][l];
                }
}

void vc::VergenceControl::showV1compResponse_1_2(){

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++){
            cv::Mat inverted;
            cv::Mat image(120, 160, CV_32FC1, &Energy[i][j]);
            //cv::imwrite("./images/disparity_maps_" + std::to_string(i) + "_" + std::to_string(j) + ".png" , image);
            cv::bitwise_not(image, inverted);
            cv::imshow("disparity maps", inverted);
            cv::waitKey(0);
        }
}

void vc::VergenceControl::getV1compResponse_full(float V1comp_resp[320][240][7][8]) {
    //------------------- GET V1 complex Response -------------------//
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 320; k++)
                for (int l = 0; l < 240; l++) {
                    V1comp_resp[k][l][i][j] = Energy[i][j][l][k];
                }
}

void vc::VergenceControl::getV1compResponse_orig(float V1comp_resp[86][86][7][8]) {
    //------------------- GET V1 complex Response -------------------//
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 8; j++)
            for (int k = 0; k < 86; k++)
                for (int l = 0; l < 86; l++) {
                    V1comp_resp[k][l][i][j] = Energy[i][j][l][k];
                }
}

void vc::VergenceControl::getFilterResponse(float LfiltRes[160][120][2][8], float RfiltResShiftE[160][120][7][8],
                                            float RfiltResShiftO[160][120][7][8]) {
    // Lfilt (2, 8, y, x); RfiltE/O(7, 8, y, x)
    //------------------- GET FILTER RESPONSES -------------------//
    for (int i = 0; i < (7); i++)
        for (int j = 0; j < (8); j++)
            for (int k = 0; k < (160); k++)
                for (int l = 0; l < (120); l++) {
                    if (i < 2) {
                        LfiltRes[k][l][i][j] = Lfilt[i][j][l][k];
                    }
                    RfiltResShiftE[k][l][i][j] = RShiftE[i][j][l][k];
                    RfiltResShiftO[k][l][i][j] = RShiftO[i][j][l][k];
                }
}

void vc::VergenceControl::getPatch(float PatchR[120][160], float PatchL[120][160]) {
    //------------------- GET PATCH IMAGES -------------------//
    // printf("%i, %i", Rpatch32F.size().width, Rpatch32F.size().height);
    for (int k = 0; k < (120); k++) {
        for (int l = 0; l < (160); l++) {
            PatchR[k][l] = Lpatch32F[k][l];
            PatchL[k][l] = Rpatch32F[k][l];
        }
    }
}

void vc::VergenceControl::getGaussian(float Gauss[86][86]) {
    //------------------- GET GAUSSIAN ARRAY -------------------//
    // printf("Gaussian: %i, %i \n", Gaussian.size().width,
    // Gaussian.size().height);
    for (int k = 0; k < (86); k++) {
        for (int l = 0; l < (86); l++) {
            Gauss[k][l] = Gaussian.at<float>(k, l);
        }
    }
}

void vc::VergenceControl::getGabor(float Gabor[2][8][80][80]) {
    //------------------- GET GABOR FIlter -------------------//
    printf("Gabor: %i, %i \n", Gfilt.Filters[0][0].size().width, Gfilt.Filters[0][0].size().height);
    for (int i = 0; i < (2); i++)
        for (int j = 0; j < (8); j++)
            for (int k = 0; k < (80); k++)
                for (int l = 0; l < (80); l++) {
                    Gabor[i][j][k][l] = Gfilt.Filters[i][j][k][l];
                }
}
