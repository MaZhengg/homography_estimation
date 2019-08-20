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

class Match{
public:
    Vec4f l1;
    Vec4f l2;
    
    double dist;
    
    Match(){
        l1 = NULL;
        l2 = NULL;
        dist = 99999;
    }
    
    Match(Vec4f line1, Vec4f line2, double distance){
        l1 = line1;
        l2 = line2;
        dist = distance;
    }
};

bool compareMatches(Match m1, Match m2){
    return (m1.dist < m2.dist);
}

/** Get center point of given line */
Vec2f getCenter(Vec4f line){
    return Vec2f( ((line[0] + line[2] )/2) , ((line[1] + line[3] )/2) );
}

/** for each L1, find the best matching L2 */
vector<Match> getBestMatches(vector<Match> matches, vector<Vec4f> templateLines){
    vector<Match> bestMatches;
    
    Match candidate;
    for(int i = 0; i < matches.size(); i++){
        if(matches[i].dist < candidate.dist){
            if(matches[i].l1 == templateLines[0]) candidate = matches[i]; // find best match for leftmost template line
        }
    }
    bestMatches.push_back(candidate);
    
    for(int i = 1; i < templateLines.size()-2; i++){
        candidate = Match();
        for(int j = 1; j < matches.size(); j++){
            if(matches[j].dist < candidate.dist){
                if(matches[j].l1 == templateLines[i]){
                    if( getCenter(matches[j].l2)[0] > getCenter(bestMatches[i-1].l2)[0] ){ // Candidate match midpoint is to the right of previous matched line
                        candidate = matches[j];
                    }
                }
            }
        }
        bestMatches.push_back(candidate);
    }
    
    candidate = Match();
    for( int i = 0; i < matches.size(); i++){
        if(matches[i].dist < candidate.dist){
            if(matches[i].l1 == templateLines[ templateLines.size()-2 ]) candidate = matches[i]; // find best match for top horizontal template line
        }
    }
    bestMatches.push_back(candidate);
    
    candidate = Match();
    for( int i = 0; i < matches.size(); i++){
        if(matches[i].dist < candidate.dist){
            if(matches[i].l1 == templateLines[ templateLines.size()-1 ]) candidate = matches[i]; // find best match for top bottom template line
        }
    }
    bestMatches.push_back(candidate);
    
    return bestMatches;
}

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

/** Get angle of given line */
double getAngle(Vec4f line1){
    double angle1 = atan2( ( line1[3] - line1[1] ), ( line1[2] - line1[0] ) );
    angle1 *= (180/ CV_PI);
    if(angle1 < 0) angle1 = 180 + angle1;
    return abs(angle1);
}

/** Return difference between angle of two given lines */
double getAngle(Vec4f line1, Vec4f line2){
    double angle1 = atan2( ( line1[3] - line1[1] ), ( line1[2] - line1[0] ) );
    double angle2 = atan2( ( line2[3] - line2[1] ), ( line2[2] - line2[0] ) );
    angle1 *= (180/ CV_PI);
    angle2 *= (180/ CV_PI);
    if(angle1 < 0) angle1 = 180 + angle1;
    if(angle2 < 0) angle2 = 180 + angle2;
    //cout << "A1: " << angle1 << " , A2: " << angle2 << endl;
    return abs(angle1-angle2);
}

/** Compare lines by angle */
bool compareVec(Vec4f v1, Vec4f v2)
{
    //return (getGradient(v1) < getGradient(v2));
    return (getAngle(v1) < getAngle(v2));
}

/** Calculate the length of a given line */
float lineLength(Vec4f line){
    return sqrt( pow((line[2] - line[0]), 2) + pow((line[1] - line[3]), 2) ) ;
}


float midpointDistance(Vec4f line1, Vec4f line2){
    Vec2f mid1 = getCenter(line1);
    Vec2f mid2 = getCenter(line2);
    return abs( lineLength( Vec4f(mid1[0], mid1[1], mid2[0], mid2[1] )));
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
    Mat dst, invdst, cdst;
    GaussianBlur( thresh, invdst, Size( 5, 5 ), 0, 0 );
    Canny(invdst, dst, 50, 200, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    
    vector<Vec4f> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 240, 250, 45 );
    
    for(int i = 0; i < lines.size(); i++ ) line( cdst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255,0,0), 2, 0);
    imshow("Lines", cdst);
    
    return lines;
}

vector<int> splitHorizontals( vector<Vec4f> lines ){
    vector<int> labels;
    
    float y1 = 0.0, y2 = 0.0;
    for(int i = 0; i < lines.size(); i++){
        y1 += lines[i][1];
        y2 += lines[i][3];
    }
    
    y1 /= lines.size();
    y2 /= lines.size();
    float avgY = (y1+y2)/2;
    
    cout << "Y threshold: " << avgY<< endl;
    for(int i = 0; i < lines.size(); i++){
        if( getCenter(lines[i])[1] < avgY ){
            labels.push_back(0);
        } else {
            labels.push_back(1);
        }
    }
    
    return labels;
}

vector<Vec4f> cleanLines(vector<Vec4f> lines){
    vector<Vec4f> sortedLines = lines;
    vector<int> sortedAngles;
    
    sort(sortedLines.begin(), sortedLines.end(), compareVec); // Sort lines by gradient to make removing duplicates easier
    for(int i = 0; i < sortedLines.size(); i++ ){
        float angle = getAngle(sortedLines[i]);
        if(angle < 10){
            sortedAngles.push_back(0);
        } else {
            sortedAngles.push_back( floor(angle/10) );
        }
    }
    
    /* Split horizontal lines into two clusters, for the bottom and top of the pitch */
    vector<Vec4f> horizontals;
    Mat labels;
    int k = 1;
    int lastOne = 0;
    int setLabel = 0;
    for(int i = 0; i < sortedAngles.size(); i++ ){
        if( i == 0){
            labels.push_back(setLabel);
            horizontals.push_back(sortedLines[i]);
        } else if( sortedAngles[i] == sortedAngles[i-1]){
            if(setLabel == 0) horizontals.push_back(sortedLines[i]);
            labels.push_back(setLabel);
        } else {
            //cout << "Angle :" << sortedAngles[i] << ", last: " << lastOne << " new: " << setLabel+1 << " new K: " << k+1 << endl;
            setLabel += 1;
            labels.push_back(setLabel);
            lastOne = setLabel;
            k += 1;
        }
    }
    
    Mat splitLabels;
    vector<int> splitVec = splitHorizontals( horizontals );
    splitLabels = Mat( splitVec );
    
    for(int i = 0; i < labels.rows; i++){
        if(labels.at<int>(i) == 0){
            labels.at<int>(i) = splitLabels.at<int>(i);
        } else {
            labels.at<int>(i) += 1;
        }
    }
    
    Mat clustered = imread("test.png");
    srand(83);
    for(int i = 0; i < k+1; i++){
        Scalar colour = Scalar( ( rand() % (int) ( 255 + 1 ) ), ( rand() % (int) ( 255 + 1 ) ), ( rand() % (int) ( 255 + 1 ) )); // Random colour for each cluster
        for(int j = 0; j < labels.rows; j++){
            if(labels.at<int>(j) == i){
                line( clustered, Point(sortedLines[j][0], sortedLines[j][1]), Point(sortedLines[j][2], sortedLines[j][3]), colour, 2, 0);
            }
        }
    }
    imshow("Clustering", clustered);
    
    vector<Vec4f> cleanedLines;
    
    for(int i = 0; i < k+1; i++){
        vector<Vec2f> points;
        
        for(int j = 0; j < labels.rows; j++){
            if(labels.at<int>(j) == i){
                points.push_back( Vec2f(sortedLines[j][0], sortedLines[j][1]));
                points.push_back( Vec2f(sortedLines[j][2], sortedLines[j][3]));
            }
        }
        Vec4f outputLine;
        fitLine(points, outputLine, DIST_L12, 0, 0.01, 0.01);
        
        // Convert from direction/point format to a line defined by its endpoints
        Vec4f pushLine = Vec4f(outputLine[2] + outputLine[0]*100, // 100 is arbitrary, line length isn't considred when converted to homogenous coordinates later.
                               outputLine[3] + outputLine[1]*100,
                               outputLine[2] - outputLine[0]*100,
                               outputLine[3] - outputLine[1]*100
                               );
        cleanedLines.push_back( pushLine );
    }
    
    return cleanedLines;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////                MAIN METHOD                 //////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


int main( int argc, char** argv){
    vector<Vec4f> rawLines = getLines("test.png");;
    Mat src = imread("test.png");
    vector<Vec4f> templateLines {Vec4f(0,0,0,800), Vec4f(430,0,430,800), Vec4f(1440,0,1440,800), Vec4f(0,0,1440,0), Vec4f(0,800,1400,800)};
    
    vector<Vec4f> lines = cleanLines(rawLines);
    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4f l = lines[i];
        line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 0);
    }
    
    for( size_t i = 0; i < templateLines.size(); i++ )
    {
         templateLines[i][0] += 50;
         templateLines[i][2] += 50;
         templateLines[i][1] += 50;
         templateLines[i][3] += 50;
    }
    
    // Find closest detected line for each template line
    vector<int> alreadyMatched;
    
    vector<Match> matches;
    
    for(int i = 0; i < templateLines.size(); i++)
    {
        for(int j = 0; j < lines.size(); j++)
        {
            float dist = midpointDistance(templateLines[i], lines[j]);
            if( getAngle(templateLines[i], lines[j]) < 70 ){
                matches.push_back( Match(templateLines[i], lines[j], dist ));
            }
        }
    }
    
    sort(matches.begin(), matches.end(), compareMatches);
    vector<Match> bestMatches = getBestMatches(matches, templateLines);
    
    vector<Vec3f> templateH = vector<Vec3f>(templateLines.size());;
    vector<Vec3f> matchedH = vector<Vec3f>(templateLines.size());;
    
    // Take detected lines and template line sets and convert them to homogenous coordinates
    for(int i = 0; i < bestMatches.size(); i++){
        templateH[i] = Vec3f(bestMatches[i].l1[0], bestMatches[i].l1[1], 1).cross( Vec3f(bestMatches[i].l1[2], bestMatches[i].l1[3], 1 ) );
        if(templateH[i][2] != 0){
            templateH[i][0] /= templateH[i][2];
            templateH[i][1] /= templateH[i][2];
            templateH[i][2] /= templateH[i][2];
        }
        
        matchedH[i] = Vec3f(bestMatches[i].l2[0], bestMatches[i].l2[1], 1).cross( Vec3f(bestMatches[i].l2[2], bestMatches[i].l2[3], 1 ) );
        if(matchedH[i][2] != 0){
            matchedH[i][0] /= matchedH[i][2];
            matchedH[i][1] /= matchedH[i][2];
            matchedH[i][2] /= matchedH[i][2];
        }
    }
    
    // Homography computation using SVD
    Mat aMat = Mat(0,9,CV_32F);
    for(int i = 0; i < templateH.size(); i++){
        float x, y, u, v;
        
        x = templateH[i][0];
        y = templateH[i][1];
        u = matchedH[i][0];
        v = matchedH[i][1];
        
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
    
    SVD aSVD = SVD(aMat);
    Mat rightSingular;
    transpose(aSVD.vt, rightSingular);
    Mat h = rightSingular.col( rightSingular.cols-1);
    
    Mat homography = Mat(3, 3, CV_32F);
    for (int i = 0 ; i < 3 ; i++){
        for (int j = 0 ; j < 3 ; j++){
            homography.at<float>(i,j) = h.at<float>(3*i+j, 0);
        }
    }
    
    //////////////////////////////////////////////////////////
    ////////////// Display output for debugging //////////////
    //////////////////////////////////////////////////////////
    
    Mat warp = imread("test.png");
    
    for(int i = 0; i < templateH.size(); i++)
    {
        line( warp, Point(bestMatches[i].l1[0], bestMatches[i].l1[1]), Point(bestMatches[i].l1[2], bestMatches[i].l1[3]), Scalar(0,255,255), 2, 0);
        line( warp, Point(bestMatches[i].l2[0], bestMatches[i].l2[1]), Point(bestMatches[i].l2[2], bestMatches[i].l2[3]), Scalar(255,0,255), 2, 0);
        Vec2f tempMid = getCenter(bestMatches[i].l1);
        Vec2f matchMid = getCenter(bestMatches[i].l2);
        line( warp, Point(tempMid[0], tempMid[1]), Point(matchMid[0], matchMid[1]), Scalar(0,255,100), 2, 0);
    }
    
    
    imshow("input", warp);
    
    Mat templateOriginal = Mat(warp.rows, warp.cols, CV_8UC3);
    Mat templateWarped = Mat(warp.rows, warp.cols, CV_8UC3);
    for(int i = 0; i < templateLines.size(); i++)
    {
        line( templateOriginal, Point(bestMatches[i].l1[0], bestMatches[i].l1[1]), Point(bestMatches[i].l1[2], bestMatches[i].l1[3]), Scalar(255,0,255), 2, 0);
    }
    warpPerspective(templateOriginal, templateWarped, homography, Size(templateOriginal.cols, templateOriginal.rows));
    for(int i = 0; i < templateH.size(); i++)
    {
        line( templateWarped, Point(bestMatches[i].l2[0], bestMatches[i].l2[1]), Point(bestMatches[i].l2[2], bestMatches[i].l2[3]), Scalar(255,255,255), 2, 0);
    }
    
    imshow("TEMPLATE", templateWarped);
    waitKey();
    
}
