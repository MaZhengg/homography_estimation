//
//  main.cpp
//  lineDetector
//
//  Created by Patrick Skinner on 24/05/19.
//  Copyright © 2019 Patrick Skinner. All rights reserved.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;


Mat src;
Mat HSV;
Mat thresh;

// Get gradient of given line
float getGradient(Vec4f v)
{
    float vGrad;
    
    if( (v[2] - v[0]) != 0 ){
        vGrad = ((v[3] - v[1] + 0.0) / (v[2] - v[0] + 0.0));
    } else {
        vGrad = 0.0;
    }
    
    return vGrad;
}


// Compare lines by gradient
bool compareVec(Vec4f v1, Vec4f v2)
{
    return (getGradient(v1) < getGradient(v2));
}


vector<Vec4f> getLines(String filename)
{
    //"chyehoo.png"
    src = imread(filename, 1);   // Read the file
    
    if(! src.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        //return vector<Vec4i> ret;
    }
    
    cvtColor(src, HSV, COLOR_BGR2HSV);
    // Detect the field based on HSV Range Values
    inRange(HSV, Scalar(32, 124, 51), Scalar(46, 255, 191), thresh);
    imshow("threshold", thresh);
    
    Mat dst, invdst, cdst;
    GaussianBlur( thresh, invdst, Size( 5, 5 ), 0, 0 );
    Canny(invdst, dst, 50, 200, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    
    vector<Vec4f> lines;
    HoughLinesP(dst, lines, 1, CV_PI/360, 240, 30, 45 );
    
    sort(lines.begin(), lines.end(), compareVec); // Sort lines by gradient to make removing duplicates easier
    
    vector<Vec4f> cleanedLines;
    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        //cout << getGradient(lines[i]) << endl;
        
        // Remove lines that are likely to be the same as the previous line
        if ( ( getGradient(lines[i]) < getGradient(lines[i+1]) + 0.05 ) && ( getGradient(lines[i]) > getGradient(lines[i+1]) - 0.05 ))
        {
            float dt = 10;
            
            if( (lines[i][0]  > lines[i+1][0] - dt) && (lines[i][0]  < lines[i+1][0] + dt) &&
               (lines[i][1]  > lines[i+1][1] - dt) && (lines[i][1]  < lines[i+1][1] + dt) &&
               (lines[i][2]  > lines[i+1][2] - dt) && (lines[i][2]  < lines[i+1][2] + dt) &&
               (lines[i][3]  > lines[i+1][3] - dt) && (lines[i][3]  < lines[i+1][3] + dt) )
            {
                cout << lines[i][0] << ",    " << lines[i+1][0] << endl;
                
                cleanedLines.push_back(lines[i]);
                i += 1;
                
            }
        }
    }
    
    return lines;
}


float lineLength(Vec4f line){
    return sqrt( pow((line[2] - line[0]), 2) + pow((line[1] - line[3]), 2) ) ;
}


float lineDistance(Vec4f line1, Vec4f line2){
    
    Vec4f ac, bd, ad, bc;
    ac = Vec4f(line1[0], line1[1], line2[0], line2[1] );
    bd = Vec4f(line1[2], line1[3], line2[2], line2[3] );
    ad = Vec4f(line1[0], line1[1], line2[2], line2[3] );
    bc = Vec4f(line1[2], line1[3], line2[0], line2[1] );
    
    return min(    max( lineLength(ac),lineLength(bd)),     max( lineLength(ad),lineLength(bc))       );
}


float getSetDistance(vector<Vec4f> templateLines, vector<Vec4f> detectedLines){
    float totalDistance = 0.0;
    
    for(int i = 0; i < templateLines.size(); i++)
    {
        for(int j = 0; j < detectedLines.size(); j++)
        {
            // For lines AB and CD, distance is defined as min(max(|𝐴𝐶|,|𝐵𝐷|),max(|𝐴𝐷|,|𝐵𝐶|))
            Vec4f ac, bd, ad, bc;
            ac = Vec4f(templateLines[i][0], templateLines[i][1], detectedLines[j][0], detectedLines[j][1] );
            bd = Vec4f(templateLines[i][2], templateLines[i][3], detectedLines[j][2], detectedLines[j][3] );
            ad = Vec4f(templateLines[i][0], templateLines[i][1], detectedLines[j][2], detectedLines[j][3] );
            bc = Vec4f(templateLines[i][2], templateLines[i][3], detectedLines[j][0], detectedLines[j][1] );
            
            totalDistance += min(    max( lineLength(ac),lineLength(bd)) ,     max( lineLength(ad),lineLength(bc))       );
        }
    }
    
    return totalDistance;
}


Mat getNullSpace(cv::Mat p)
{
    SVD svd = cv::SVD(p, cv::SVD::FULL_UV);
    Mat vt_ = svd.vt;
    int i;
    for (i = 1; i <= 3; i++)
    {
        if (p.at<double>(i - 1, i - 1) == 0)
        {
            break;
        }
    }
    cv::Mat result = vt_(cv::Rect(0, i-1, p.cols, vt_.rows-i+1));
    cv::Mat result_t;
    cv::transpose(result, result_t);
    return result_t;
}


int main( int argc, char** argv )
{
    vector<Vec4f> lines = getLines("chyehoo2.png");
    //lines.push_back(Vec4f(80+120,95+130,120,600+130));
    vector<Vec4f> lines2 = getLines("chyehoo.png");
    Mat src = imread("chyehoo2.png", 1);
    Mat src2 = imread("chyehoo.png", 1);
    
    vector<Point2f> srcPoints, src2Points;
    int n = 0;
    
    // Draw detected lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4f l = lines[i];
        line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 0);
        
        
        float xStep = (l[2] - l[0])/10;
        float yStep = (l[3] - l[1])/10;
        
        for(int j = 0; j < 10; j++){
            srcPoints.push_back(   Point( l[0]+xStep, l[1]+yStep )     );
            xStep += (l[2] - l[0])/10;
            yStep += (l[3] - l[1])/10;
        }
        /*
        srcPoints.push_back( Point(l[0], l[1]) );
        srcPoints.push_back( Point(l[2], l[3]) );
        srcPoints.push_back( Point( l[0]+(l[2]/2) , l[1]+(l[3]/2) ) );
        */
        n += 2;
    }
    
    for( size_t i = 0; i < lines2.size(); i++ )
    {
        Vec4f l = lines2[i];
        line( src2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 0);
        
        float xStep = (l[2] - l[0])/10;
        float yStep = (l[3] - l[1])/10;
        
        for(int j = 0; j < 10; j++){
            src2Points.push_back(   Point( l[0]+xStep, l[1]+yStep )     );
            xStep += (l[2] - l[0])/10;
            yStep += (l[3] - l[1])/10;
        }
        /**
        src2Points.push_back( Point(l[0], l[1]) );
        src2Points.push_back( Point(l[2], l[3]) );
        src2Points.push_back( Point( l[0]+(l[2]/2) , l[1]+(l[3]/2) ) );
         */
        n += 2;
    }
    
    //src2Points.push_back(   Point(0, 0)     );
    
    for(int i = 0; i < srcPoints.size(); i++){
        //circle(src, srcPoints[i], 3, Scalar(255,0,0));
    }
    for(int i = 0; i < src2Points.size(); i++){
        //circle(src2, src2Points[i], 3, Scalar(255,0,0));
    }
    
    cout << "Points: "<< srcPoints.size() << ", " << src2Points.size() << endl;
    //cout << "Check: "<< srcPoints.checkVector(2) << ", " << src2Points.checkVector(2) << endl;
    
    vector<Vec4f> templateLines { Vec4f(0,0,550,0), Vec4f(0,600,550,600),  Vec4f(550,0,550,600), Vec4f(0,600,550,0), Vec4f(0,0,0,600)};
    cout << "Set Dist: " << getSetDistance(templateLines, lines) << endl;
    
    for( size_t i = 0; i < templateLines.size(); i++ )
    {
        
        templateLines[i][0] += 120;
        templateLines[i][2] += 120;
        
        templateLines[i][1] += 130;
        templateLines[i][3] += 130;
        
        Vec4f l = templateLines[i];
        line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,255), 2, 0);
    }
    
    cout << "Set Dist: " << getSetDistance(templateLines, lines) << endl;
    
    Mat matched = imread("chyehoo2.png", 1);
    
    
    vector<Vec4f> templateMatches(templateLines.size());
    for(int i = 0; i < templateLines.size(); i++)
    //for(int i = 0; i < 4; i++)
    {
        Vec4f closest;
        float closestDist = 9999;
        
        for(int j = 0; j < lines.size(); j++)
        {
            if( lineDistance(templateLines[i], lines[j]) < closestDist)
            {
                closest = lines[j];
                closestDist = lineDistance(templateLines[i], lines[j]);
            }
        }
        
        line( matched, Point(closest[0], closest[1]), Point(closest[2], closest[3]), Scalar(255,0,255), 2, 0);
        cout << "closest dist: " << closestDist << endl;
        templateMatches[i] = closest;
        
        Vec2f tempMid = Vec2f( ((templateLines[i][0] + templateLines[i][2] )/2) , ((templateLines[i][1] + templateLines[i][3] )/2) );
        Vec2f matchMid = Vec2f( ((closest[0] + closest[2] )/2) , ((closest[1] + closest[3] )/2) );
        //line( src, Point(tempMid[0], tempMid[1]), Point(matchMid[0], matchMid[1]), Scalar(255,255,255), 1, 0);
    }
    
    imshow("src", src);
    
    vector<Vec3f> set1 = vector<Vec3f>(templateLines.size());
    for(int i = 0; i < templateLines.size(); i++){
        set1[i] = Vec3f(templateLines[i][0], templateLines[i][1], 1).cross( Vec3f(templateLines[i][2], templateLines[i][3], 1 ) );
        if(set1[i][2] != 0){
            set1[i][0] /= set1[i][2];
            set1[i][1] /= set1[i][2];
            set1[i][2] /= set1[i][2];
        }
        cout << "Line 1: " << set1[i] << endl;
    }
    
    vector<Vec3f> set2 = vector<Vec3f>(templateMatches.size());
    for(int i = 0; i < templateMatches.size(); i++){
        set2[i] = Vec3f(templateMatches[i][0], templateMatches[i][1], 1).cross( Vec3f(templateMatches[i][2], templateMatches[i][3], 1 ) );
        if(set2[i][2] != 0){
            set2[i][0] /= set2[i][2];
            set2[i][1] /= set2[i][2];
            set2[i][2] /= set2[i][2];
        }
        cout << "Line 2: " << set2[i] << endl;
    }
    
    /*
    Point2f centroid = Point2f(0.0, 0.0);
    for(int i = 0; i < templateLines.size(); i++){
        centroid.x += ((templateLines[i][0] + templateLines[i][2] + templateMatches[i][0] + templateMatches[i][2] )/4);
        centroid.y += ((templateLines[i][1] + templateLines[i][3] + templateMatches[i][1] + templateMatches[i][3] )/4);
    }
    
    centroid.x /= templateLines.size();
    centroid.y /= templateLines.size();
    
    cout << "Centroid of points: " << centroid << endl;
    */
    
    
    Mat aMat = Mat(0,9,CV_32F);
    
    for(int i = 0; i < templateMatches.size(); i++){
        float x, y, u, v;
        
        x = set1[i][0];
        y = set1[i][1];
        u = set2[i][0];
        v = set2[i][1];
        
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
    
    if(templateLines.size() == 4){
        //aMat.resize(9, cv::Scalar(0));
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
    
    cout << "Homography: " << homography << endl << endl;
    
    Mat warp = imread("chyehoo.png");
    Mat warped;
    
    
    warpPerspective(warp, warped, homography, Size(warp.cols, warp.rows));
    
    
    for(int i = 0; i < templateMatches.size(); i++)
    {
        line( warped, Point(templateMatches[i][0], templateMatches[i][1]), Point(templateMatches[i][2], templateMatches[i][3]), Scalar(255,0,255), 2, 0);
    }
    
    imshow("fuggg :DDDD", warped);
    
    imshow("matched", matched);
    waitKey();
    
    return 0;
}


