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

#ifndef LIB_VERGENCECONTROL_H
#define LIB_VERGENCECONTROL_H

#include <yarp/sig/all.h>

#include <ctime>
#include <opencv2/opencv.hpp>
#include <string>

namespace vergencecontrol {

//----------------------------------------------------------
// A simple wrapper class for the population coding model
// of vergence control computation, developed in C on FFT convolution
//----------------------------------------------------------
class VergenceControl {
    int width;     // image width
    int height;    // image height

 public:
    VergenceControl(int width, int height, const std::string &filter_filename);
    ~VergenceControl();

    // Read left and right images from file Load_Img_File
    void loadImgFile(const std::string &img_filename, char c);
    void loadImg(yarp::sig::ImageOf<yarp::sig::PixelMono> &img, char c);
    void loadImgArr(int rows, int cols, float *img, char c);
    void loadMatImg(cv::Mat&img, char c);


    // Compute vergence control
    void computeFilterPatch();

    void setCenter(int x, int y);
    void getCenter(int *x, int *y);
    void getSubImgSize(int *M, int *N);

    void getV1compResponse_1_4(float V1comp_resp[80][60][7][8]);
    void getV1compResponse_1_2(float V1comp_resp[160][120][7][8]);
    void getV1compResponse_1_2_(std::vector<cv::Mat>&V1comp_resp);
    void getV1compResponse_full(float V1comp_resp[320][240][7][8]);
    void getV1compResponse_full_(std::vector<cv::Mat>&V1comp_resp);
    void getV1compResponse_orig(float V1comp_resp[86][86][7][8]);
    void showV1compResponse_1_2();
    void showV1compResponse_full();

    void getFilterResponse(float LfiltRes[160][120][2][8], float RfiltResShiftE[160][120][7][8], float RfiltResShiftO[160][120][7][8]);
    void getPatch(float PatchR[120][160], float PatchL[120][160]);
    void getGaussian(float Gauss[86][86]);
    void getGabor(float Gabor[2][8][80][80]);

 private:
    // GABOR FILTERS
    typedef struct {
        int Nori, Nph;
        float *theta;
        float *phase;
        float phi[2];

        float sigma;
        float f;
        // int taps;
        // int HalfSize;

        int taps;

        int HalfSize;

        float B;

        // cv::Mat* EvenFilters;
        // cv::Mat* OddFilters;
        cv::Mat1f *Filters[2];
        cv::Mat1i X, Y;
    } Gabor;

    Gabor Gfilt;
    int M, N, Mcut, Ncut;
    int Mpatch, Npatch;
    int Mpad, Npad;
    cv::Mat L, R, Lpad, Rpad, Lpatch, Rpatch;
    cv::Mat1f Lpatch32F, Rpatch32F;

    cv::Mat1f *Lfilt[2], *Rfilt[2];                    // Even/Odd filtered images
    cv::Mat1f *RShiftE[7], *RShiftO[7], *Energy[7];    // Even/Odd filtered and phase shifted right image

    cv::Mat1f tmpEven, tmpOdd, tmpSum;
    cv::Mat SUM_H, SUM_V, SUM_N;
    float fovea_size;
    cv::Mat Gaussian;
    cv::Point2d Center;
    cv::Rect_<double> ROI;

    std::clock_t start_time, end_time;

    cv::Mat planes[2];
    cv::Mat tmpF;

    void createGaborFilters();
    void loadGaborParams(const std::string &ini_filter_file);
    void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y);
    void meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y);
    void Gabor2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G, float f, float theta, float sigma, float phi);

    void create_Gaussian(const std::string &ini_filter_file);
    void Gaussian2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G, float sigma);

    void filtGaborBank();
    void shiftGaborBank();
    void computeEnergy();
};

}    // namespace vergencecontrol

#endif
