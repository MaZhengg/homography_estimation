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

/** Get gradient of given line */
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

/** Compare lines by gradient */
bool compareVec(Vec4f v1, Vec4f v2)
{
    return (getGradient(v1) < getGradient(v2));
}

double getAngle(Vec4f line1, Vec4f line2){
    double angle1 = atan2( ( line1[3] - line1[1] ), ( line1[2] - line1[0] ) );
    double angle2 = atan2( ( line2[3] - line2[1] ), ( line2[2] - line2[0] ) );
    angle1 *= (180/ CV_PI);
    angle2 *= (180/ CV_PI);
    if(angle1 < 0) angle1 = 180 + angle1;
    if(angle2 < 0) angle2 = 180 + angle2;
    cout << "A1: " << angle1 << " , A2: " << angle2 << endl;
    return abs(angle1-angle2);
}

/** Return vector of all detected Hough lines in given image */
vector<Vec4f> getLines(String filename)
{
    src = imread(filename, 1);   // Read the file
    
    if(! src.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
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
    HoughLinesP(dst, lines, 1, CV_PI/180, 240, 30, 45 );
    
    /*
    sort(lines.begin(), lines.end(), compareVec); // Sort lines by gradient to make removing duplicates easier
    
    vector<Vec4f> cleanedLines;
    for( size_t i = 0; i < lines.size(); i++ )
    {
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
    */
    
    /*
    RNG rng;
    
    vector<vector<Point> > contours;
    findContours( thresh, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<vector<Point> > contours_poly( contours.size() );
    Rect boundRect;
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    int biggest = 0;
    
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        if(boundingRect( contours_poly[i] ).area() > biggest){
            boundRect = boundingRect( contours_poly[i] );
            biggest = boundingRect( contours_poly[i] ).area();
        }
    }
    Mat drawing = Mat::zeros( thresh.size(), CV_8UC3 );
    Scalar color = Scalar( 255, 255, 0 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        drawContours( drawing, contours_poly, (int)i, color );
    }
    color = Scalar(0, 255, 0);
    rectangle( drawing, boundRect.tl(), boundRect.br(), color, 2 );
    imshow( "Contours", drawing );
    
    
    vector<vector<Point>> hull( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        convexHull( contours[i], hull[i] );
    }
    drawing = Mat::zeros( dst.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        //drawContours( drawing, contours, (int)i, color );
        drawContours( drawing, hull, (int)i, color );
    }
    imshow( "Hull demo", drawing );
    
    */
    
    
    
    return lines;
}

/** Calculate the length of a line */
float lineLength(Vec4f line){
    return sqrt( pow((line[2] - line[0]), 2) + pow((line[1] - line[3]), 2) ) ;
}

/** Calculate the Hausdorff distance between two lines */
float lineDistance(Vec4f line1, Vec4f line2){
    
    Vec4f ac, bd, ad, bc;
    ac = Vec4f( line1[0], line1[1], line2[0], line2[1] );
    bd = Vec4f( line1[2], line1[3], line2[2], line2[3] );
    ad = Vec4f( line1[0], line1[1], line2[2], line2[3] );
    bc = Vec4f( line1[2], line1[3], line2[0], line2[1] );
    
    return min(    max( lineLength(ac),lineLength(bd)),     max( lineLength(ad),lineLength(bc))       );
}

/** Calculate the total Hausdorff distance between two line sets */
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


int main( int argc, char** argv )
{
    vector<Vec4f> lines = getLines("test.png");
    //lines.push_back(Vec4f(80+120,95+130,120,600+130));
    vector<Vec4f> lines2 = getLines("test.png");
    Mat src = imread("test.png", 1);
    Mat src2 = imread("test.png", 1);

    // Draw detected lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4f l = lines[i];
        line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 0);
    }
    
    for( size_t i = 0; i < lines2.size(); i++ )
    {
        Vec4f l = lines2[i];
        line( src2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 0);
    }
    
    imshow("wtf", src);
    
    //vector<Vec4f> templateLines { Vec4f(0,0,550,0), Vec4f(0,600,550,600),  Vec4f(550,0,550,600), Vec4f(0,600,550,0), Vec4f(0,0,0,600)};
    vector<Vec4f> templateLines { Vec4f(0,0,1440,0), Vec4f(0,800,1400,800),  Vec4f(1440,0,1440,800), Vec4f(0,0,0,800), Vec4f(430,0,430,800)};
    //vector<Vec4f> templateLines = lines;
    
    for( size_t i = 0; i < templateLines.size(); i++ )
    {
        
        templateLines[i][0] += 120;
        templateLines[i][2] += 120;
        
        templateLines[i][1] += 130;
        templateLines[i][3] += 130;
        
        Vec4f l = templateLines[i];
        line( src2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,255), 2, 0);
    }
    
    Mat matched = imread("test.png", 1);
    
    // Find closest detected line for each template line
    vector<int> alreadyMatched;
    vector<Vec4f> templateMatches(templateLines.size());
    for(int i = 0; i < templateLines.size(); i++)
    {

        Vec4f closest;
        float closestDist = 999999;
        
        for(int j = 0; j < lines2.size(); j++)
        {
            if( lineDistance(templateLines[i], lines2[j]) < closestDist)
            {
                //cout << i << ", " << j << " = " << lineDistance(templateLines[i], lines2[j])<< endl;
                if( getAngle(templateLines[i], lines2[j]) < 80 ){
                    bool flag = false;
                    for(int n = 0; n < alreadyMatched.size(); n++){
                        if(j == alreadyMatched[n]){
                            flag = true;
                        }
                    }
                    
                    if(!flag){
                        closest = lines2[j];
                        closestDist = lineDistance(templateLines[i], lines2[j]);
                        alreadyMatched.push_back(j);
                    }
                }
            }
        }
        line( matched, Point(closest[0], closest[1]), Point(closest[2], closest[3]), Scalar(255,0,255), 2, 0);
        templateMatches[i] = closest;
        
        // Draw connections between matched line pairs
        
        Vec2f tempMid = Vec2f( ((templateLines[i][0] + templateLines[i][2] )/2) , ((templateLines[i][1] + templateLines[i][3] )/2) );
        Vec2f matchMid = Vec2f( ((closest[0] + closest[2] )/2) , ((closest[1] + closest[3] )/2) );
        line( src2, Point(tempMid[0], tempMid[1]), Point(matchMid[0], matchMid[1]), Scalar(0,255,100), 2, 0);
        
    }
    
    for(int i = 0; i < templateLines.size(); i++){
        cout << templateLines[i] << " to " << templateMatches[i] << ",    angle: " << getAngle(templateLines[i], templateMatches[i]) << endl;;
    }
    
    imshow("src2", src2);
    
    // Take template lines and convert them to homogenous coordinates
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
    
    // Take detected lines and convert them to homogenous coordinates
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
    
    cout << "\n\n";
    
    // Calculate centroid of both point sets
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
    
    Mat warp = imread("test.png");
    Mat warped;
   
    warpPerspective(warp, warped, homography, Size(warp.cols, warp.rows));
    
    
    for(int i = 0; i < templateMatches.size(); i++)
    {
        line( warped, Point(templateMatches[i][0], templateMatches[i][1]), Point(templateMatches[i][2], templateMatches[i][3]), Scalar(255,0,255), 2, 0);
    }
    
    imshow("fuggg :DDDD", warped);
    
    imshow("matched", matched);
    
    Mat templateOriginal = Mat(warp.rows, warp.cols, CV_8UC3);
    Mat templateWarped = Mat(warp.rows, warp.cols, CV_8UC3);
    for(int i = 0; i < templateLines.size(); i++)
    {
        line( templateOriginal, Point(templateLines[i][0], templateLines[i][1]), Point(templateLines[i][2], templateLines[i][3]), Scalar(255,0,255), 2, 0);
    }
    warpPerspective(templateOriginal, templateWarped, homography, Size(templateOriginal.cols, templateOriginal.rows));
    for(int i = 0; i < templateMatches.size(); i++)
    {
        line( templateWarped, Point(templateMatches[i][0], templateMatches[i][1]), Point(templateMatches[i][2], templateMatches[i][3]), Scalar(255,255,255), 2, 0);
    }
    imshow("TEMPLATE", templateWarped);
    
    waitKey();
    
    return 0;
}


