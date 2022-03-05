//
// Created by tyy on 2021/11/8.
//

#ifndef CAMCALIBRATION_CALIBRATION_H
#define CAMCALIBRATION_CALIBRATION_H
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <errno.h>

using namespace std;
using namespace cv;

class tool{
public:
    ~tool();
    void calibration(string path, string output_file);

    int amount;
    vector<string> files;

private:
    bool getFileName(string path,vector<string> &files);
    void check();

    Size img_size;
    Size board_size = Size (8,11); //棋盘格角点个数
    Size square_size = Size(29.82,27.91); //实际正方形尺寸 mm
    int count;
    vector<Point2f> img_p_buf; //角点缓存
    vector<vector<Point2f>> img_p_det; //检测到的角点
    vector<vector<Point3f>> obj_p; //世界坐标下的点  x y z
    Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));
    Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0));
    vector<Mat> tvec;
    vector<Mat> rvec;
};

#endif //CAMCALIBRATION_CALIBRATION_H
