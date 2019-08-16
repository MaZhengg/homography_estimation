//
//  HomographyFromPoints.cpp
//  Calculates a homography between two point sets using DLT algorithm
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
    Mat src = imread("test.png", 1);
    Mat src2 = imread("test2.png", 1);
    
    //vector<Vec4f> templateLines { Vec4f(0,0,550,0), Vec4f(0,600,550,600), Vec4f(0,0,0,600), Vec4f(550,0,550,600), Vec4f(0,600,550,0) };
    
    vector<Vec2f> set1 { Vec2f(0, 0), Vec2f(550, 0), Vec2f(0, 600), Vec2f(550, 600)};
    vector<Vec2f> set2 { Vec2f(80, 95), Vec2f(550, 0), Vec2f(0, 600), Vec2f(550-25, 600-50)};
    
    
    Mat aMat = Mat(0,9,CV_32F);
    
    for(int i = 0; i < set1.size(); i++){
        float x, y, u, v;
        
        x = set1[i][0];
        y = set1[i][1];
        u = set2[i][0];
        v = set2[i][1];

        Mat a1 = Mat(2, 9, CV_32F);
        
        a1.at<float>(0,0) = -x;
        a1.at<float>(0,1) = -y;
        a1.at<float>(0,2) = -1;
        a1.at<float>(0,3) = 0;
        a1.at<float>(0,4) = 0;
        a1.at<float>(0,5) = 0;
        a1.at<float>(0,6) = u*x;
        a1.at<float>(0,7) = u*y;
        a1.at<float>(0,8) = u;
        
        a1.at<float>(1,0) = 0;
        a1.at<float>(1,1) = 0;
        a1.at<float>(1,2) = 0;
        a1.at<float>(1,3) = -x;
        a1.at<float>(1,4) = -y;
        a1.at<float>(1,5) = -1;
        a1.at<float>(1,6) = v*x;
        a1.at<float>(1,7) = v*y;
        a1.at<float>(1,8) = v;
        
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
    
    Mat homo;
    homo = findHomography(Mat(set1), Mat(set2), 0);
    Mat distort;
    warpPerspective(src, distort, homo, Size(805, 871) );
    Mat distort2;
    warpPerspective(src, distort2, homography, Size(805, 871) );
    cout << "Homo: " << homo << endl << endl;
    cout << "Homo2: " << homography << endl;

    
    imshow("src2", src2);
    imshow("distorted with OCV", distort);
    imshow("distorted with DLT", distort2);
    waitKey();
    
    
}
