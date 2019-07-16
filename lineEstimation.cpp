//
//  main.cpp
//  homographytest
//
//  Created by Patrick Skinner on 15/07/19.
//  Copyright © 2019 Patrick Skinner. All rights reserved.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, const char * argv[]) {
    Mat src = imread("chyehoo.png", 1);
    Mat src2 = imread("chyehoo2.png", 1);
    
    vector<Vec4f> set1 { Vec4f(0,0,550,0), Vec4f(0,600,550,600), Vec4f(0,0,0,600), Vec4f(550,0,550,600), Vec4f(0,600,550,0) };
    vector<Vec4f> set2 { Vec4f(0,0,550,0), Vec4f(0,600,550,600), Vec4f(0,0,0,600), Vec4f(550,0,550,600), Vec4f(0,600,550,0) };
    
    //vector<Vec4f> set2 { Vec4f(80,95,550,0), Vec4f(0,600,525,550), Vec4f(0,0,0,600), Vec4f(550,0,525,550), Vec4f(0,600,550,0) };
    
    vector<Vec3f> lines1 = vector<Vec3f>(set1.size());
    for(int i = 0; i < set1.size(); i++){
        lines1[i] = Vec3f(set1[i][0], set1[i][1], 1).cross( Vec3f(set1[i][2], set1[i][3], 1 ) );
        cout << "Line 1: " << lines1[i] << endl;
    }
    
    vector<Vec3f> lines2 = vector<Vec3f>(set1.size());
    for(int i = 0; i < set2.size(); i++){
        lines2[i] = Vec3f(set2[i][0], set2[i][1], 1).cross( Vec3f(set2[i][2], set2[i][3], 1 ) );
        cout << "Line 2: " << lines2[i] << endl;
    }
    
    Mat aMat = Mat(0,9,CV_32F);
    
    for(int i = 0; i < set1.size(); i++){
        float x, y, u, v;
        
        
        x = lines1[i][0];
        y = lines1[i][1];
        u = lines2[i][0];
        v = lines2[i][1];

        Mat a1 = Mat(2, 9, CV_32F);
        
        a1.at<float>(0,0) = -u;
        a1.at<float>(0,1) = 0;
        a1.at<float>(0,2) = u*x;
        a1.at<float>(0,3) = -v;
        a1.at<float>(0,4) = 0;
        a1.at<float>(0,5) = v*x;
        a1.at<float>(0,6) = -1;
        a1.at<float>(0,7) = 0;
        a1.at<float>(0,8) = x;
        
        a1.at<float>(1,0) = 0;
        a1.at<float>(1,1) = -u;
        a1.at<float>(1,2) = u*y;
        a1.at<float>(1,3) = 0;
        a1.at<float>(1,4) = -v;
        a1.at<float>(1,5) = v*y;
        a1.at<float>(1,6) = 0;
        a1.at<float>(1,7) = -1;
        a1.at<float>(1,8) = y;
        
        vconcat(aMat, a1, aMat);
    }
    
    if(set1.size() == 4){
        aMat.resize(9, cv::Scalar(0));
    }
    cout << "aMat size = " << aMat.size << "\n" ;
    //cout << "aMat  = " << aMat << "\n\n" ;
    
    SVD aSVD = SVD(aMat);
    Mat rightSingular;
    transpose(aSVD.vt, rightSingular);
    
    cout << rightSingular << endl << endl;
    
    cout << "rs size = " << rightSingular.size << "\n" ;
    
    Mat h = rightSingular.col( rightSingular.cols-1);
    cout << "h size = " << h.size << "\n" ;
    //cout << "h = " << h << "\n\n" ;
    
    Mat homography = Mat(3, 3, CV_32F);
    for (int i = 0 ; i < 3 ; i++){
        for (int j = 0 ; j < 3 ; j++){
            homography.at<float>(i,j) = h.at<float>(3*i+j, 0);
            //cout << 3*i+j << ", " ;
        }
    }
    
    cout << "\n\n" ;
    
    Mat distort;
    warpPerspective(src, distort, homography, Size(805, 871) );
    cout << "Homography: " << homography << endl;

    
    imshow("src2", src2);
    imshow("distorted with DLT using lines", distort);
    waitKey();
    
    
}
