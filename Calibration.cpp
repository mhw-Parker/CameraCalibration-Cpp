//
// Created by tyy on 2021/11/8.
//
#include "Calibration.h"

tool::~tool() {}

bool tool::getFileName(string path, vector<string> &files)
{
    path = path.append("/");
    DIR *p_dir;
    const char *str = path.c_str();
    p_dir = opendir(str);
    if(p_dir == NULL){
        cout << "can't open" << path << endl;
        return false;
    }
    else{
        struct dirent *p_dirent;
        while(p_dirent = readdir(p_dir)){
            string tmp_file = p_dirent->d_name;
            if(tmp_file == "." || tmp_file == ".."){
                continue;
            }
            else{
                files.push_back(tmp_file);
            }
        }
        closedir(p_dir);
        amount = files.size();
        return true;
    }
}

void tool::calibration(string path, string output_file) {
    std::ofstream clean(output_file,ios::trunc); //清空txt
    std::ofstream logWrite(output_file,ios::out); //可以写入txt
    getFileName(path,files); //files 里存放了图片的名字
//    for(int i=1;i<=files.size();i++)
//        logWrite << files[i] << endl;
    path = path.append("/");
    int i, j, t;
    for(i = 0; i < files.size(); i++){
        string file_name = path + files[i];
        Mat img = imread(file_name);
        imshow("origin",img);
        waitKey(1);
        if(i==1){
            img_size.width = img.cols;
            img_size.height = img.rows;
        }
        if(!findChessboardCorners(img,board_size,img_p_buf)){
            cout << "can't find chessboard point " << endl;
            continue;
        }
        else{
            Mat gray;
            cvtColor(img,gray,COLOR_RGB2GRAY);
            cornerSubPix(gray, img_p_buf,
                         Size(10,10),
                         Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03));
            //find4QuadCornerSubpix(gray,img_p_buf,Size(11,11));
            img_p_det.push_back(img_p_buf);
            drawChessboardCorners(gray,board_size,img_p_buf, true);
            imshow("Camera Calibration",gray);
            waitKey(1);
        }

    }
    int corner_num = board_size.width * board_size.height;
    for(int i = 0; i < files.size(); i++){
        vector<Point3f> tmp_p;
        for(j = 0; j < board_size.height; j++){
            for(t = 0; t < board_size.width; t++){
                Point3f real_p;
                real_p.x = t * square_size.width;
                real_p.y = j * square_size.height;
                real_p.z = 0;
                tmp_p.push_back(real_p);
            }
        }
        obj_p.push_back(tmp_p);
    }

    calibrateCamera(obj_p,img_p_det,img_size,cameraMatrix,distCoeffs,rvec,tvec,0);

    double total_err = 0;
    double err = 0;
    vector<Point2f> img_p_cal; //重投影点
    for(i = 0; i < files.size(); i++){
        vector<Point3f> tmp_p = obj_p[i]; //存放当前图片实际的世界坐标
        projectPoints(tmp_p, rvec[i],tvec[i],cameraMatrix,distCoeffs,img_p_cal); //计算反投影点的位置
        vector<Point2f> tmp_img_p = img_p_det[i]; //存放当前图片的检测角点
        Mat tmp_img_p_mat = Mat(1,tmp_img_p.size(),CV_32FC2);
        Mat img_p2mat = Mat(1,img_p_cal.size(),CV_32FC2);
        for (int j = 0; j < tmp_img_p.size(); j++) {
            img_p2mat.at<Vec2f>(0,j) = Vec2f(img_p_cal[j].x, img_p_cal[j].y);
            tmp_img_p_mat.at<Vec2f>(0,j) = Vec2f (tmp_img_p[j].x, tmp_img_p[j].y);
        }
        err = norm(img_p2mat, tmp_img_p_mat, NORM_L2);
        total_err += err /= corner_num;
        cout << "第 " << i << " 张平均像素误差：" << err << "像素" << endl;
    }
    cout << "Camera Intrinsic Matrix : " << endl << cameraMatrix << endl;
    cout << "Camera distortion coefficients : " << endl << distCoeffs << endl;
}

void tool::check()
{

}
