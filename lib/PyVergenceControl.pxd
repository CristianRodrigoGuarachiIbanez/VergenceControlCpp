from libcpp.string cimport string

cdef extern from "yarp/sig/all.h" :
    pass

cdef extern from "../VergenceControl.h" namespace "vergencecontrol":
    cdef cppclass VergenceControl:
        VergenceControl(int width, int height, const string &filter_filename) except +

        void loadImgArr(int rows, int cols, float *img, char c);

        void getV1compResponse_1_2(float V1comp_resp[160][120][7][8]);
        void getFilterResponse(float LfiltRes[160][120][2][8], float RfiltResShiftE[160][120][7][8], float RfiltResShiftO[160][120][7][8]);
        void getPatch(float PatchR[120][160], float PatchL[120][160]);
        void getGaussian(float Gauss[86][86]);
        void getGabor(float Gabor[2][8][80][80]);

        void filtGaborBank();
        void shiftGaborBank();
        void computeEnergy();
