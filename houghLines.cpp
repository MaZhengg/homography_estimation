//
//  main.cpp
//  lineDetector
//
//  Created by Patrick Skinner on 24/05/19.
//  Copyright © 2019 Patrick Skinner. All rights reserved.
//*

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
        int l1;
        int l2;

        double dist;
    
        Match(int line1, int line2, double distance){
            l1 = line1;
            l2 = line2;
            dist = distance;
        }
    
    Match(){
        l1 = NULL;
        l2 = NULL;
        dist = 99999;
    }
    
    // Add way to sort matches
};

bool compareMatches(Match m1, Match m2){
    return (m1.dist < m2.dist);
}

vector<Match> getBestMatches(vector<Match> matches){
    vector<Match> bestMatches;
    vector<int> l1s;
    vector<int> l2s;
    
    for(int i = 0; i < matches.size(); i++){
        //cout << matches[i].dist << endl;
        
        /*
         if match[i].l1 isnt already matched
            and match[i].l2 isnt already matched
                Add match to list
                Add l1 to matched templateLines
                Add l2 to matched lines
        */
        bool flag = false;
        for(int j = 0; j < l1s.size(); j++){
            if(l1s[j] == matches[i].l1 || l2s[j] == matches[i].l2){
                flag = true;
            }
        }
        
        if(!flag){
            bestMatches.push_back(matches[i]);
            l1s.push_back(matches[i].l1 );
            l2s.push_back(matches[i].l2 );
        }
    }
    
    return bestMatches;
}

Vec2f getCenter(Vec4f line){
    return Vec2f( ((line[0] + line[2] )/2) , ((line[1] + line[3] )/2) );
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

/** Compare lines by gradient */
bool compareVec(Vec4f v1, Vec4f v2)
{
    //return (getGradient(v1) < getGradient(v2));
    return (getAngle(v1) < getAngle(v2));
}

bool isEqual(const Vec4f& _l1, const Vec4f& _l2)
{
    Vec4f l1(_l1), l2(_l2);
    
    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
    
    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);
    
    if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
        return false;
    
    float mx1 = (l1[0] + l1[2]) * 0.5f;
    float mx2 = (l2[0] + l2[2]) * 0.5f;
    
    float my1 = (l1[1] + l1[3]) * 0.5f;
    float my2 = (l2[1] + l2[3]) * 0.5f;
    float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));
    
    if (dist > std::max(length1, length2) * 0.5f)
        return false;
    
    return true;
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

vector<int> splitHorizontals( vector<Vec4f> lines ){
    vector<int> labels;
    
    float y1 = 0.0, y2 = 0.0;
    for(int i = 0; i < lines.size(); i++){
        cout << lines[i][1] << " - " << lines[i][3] << endl;
        y1 += lines[i][1];
        y2 += lines[i][3];
    }
    
    y1 /= lines.size();
    y2 /= lines.size();
    float avgY = (y1+y2)/2;
    
    cout << "Y threshold: " << avgY<< endl;
    for(int i = 0; i < lines.size(); i++){
        if( getCenter(lines[i])[1] < avgY ){
            //cout << "above: " << getCenter(lines[i])[1] << ", ";
            labels.push_back(0);
        } else {
            //cout << "below: " << getCenter(lines[i])[1] << ", ";
            labels.push_back(1);
        }
    }
    
    return labels;
}

void findHull(Mat thresh){
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
    drawing = Mat::zeros( thresh.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        //drawContours( drawing, contours, (int)i, color );
        drawContours( drawing, hull, (int)i, color );
    }
    imshow( "Hull demo", drawing );
}

/** Return vector of all detected Hough lines in given image */
vector<Vec4f> getLines(String filename)
{
    src = imread(filename, 1);   // Read the file
    
    if(! src.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
    }
    Mat cleaned = src;
    
    cvtColor(src, HSV, COLOR_BGR2HSV);
    // Detect the field based on HSV Range Values
    inRange(HSV, Scalar(32, 124, 51), Scalar(46, 255, 191), thresh);
    imshow("threshold", thresh);
    
    Mat dst, invdst, cdst;
    GaussianBlur( thresh, invdst, Size( 5, 5 ), 0, 0 );
    Canny(invdst, dst, 50, 200, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    imshow("canny", dst);
    vector<Vec4f> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 240, 250, 45 );
    
  ///////////////////////////////////////////////////////////////////////////////////
    
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
        cout << angle << " label: " << floor(angle/10) << endl;
    }
    
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
            cout << "Angle :" << sortedAngles[i] << ", last: " << lastOne << " new: " << setLabel+1 << " new K: " << k+1 << endl;
            setLabel += 1;
            labels.push_back(setLabel);
            lastOne = setLabel;
            k += 1;
        }
    }
    Mat centers;
    Mat splitLabels;
    vector<int> splitVec = splitHorizontals( horizontals );
    splitLabels = Mat( splitVec );
    //kmeans(horizontals, 2, splitLabels, TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0), 1, KMEANS_PP_CENTERS, centers);
    cout << splitLabels << endl << endl;
    for(int i = 0; i < labels.rows; i++){
        if(labels.at<int>(i) == 0){
            labels.at<int>(i) = splitLabels.at<int>(i);
        } else {
            labels.at<int>(i) += 1;
        }
    }
    
    for(int i = 0; i < k+1; i++){
        Scalar colour = Scalar( ( rand() % (int) ( 255 + 1 ) ), ( rand() % (int) ( 255 + 1 ) ), ( rand() % (int) ( 255 + 1 ) )); // Random colour for each cluster
        for(int j = 0; j < labels.rows; j++){
            if(labels.at<int>(j) == i){
                line( cleaned, Point(sortedLines[j][0], sortedLines[j][1]), Point(sortedLines[j][2], sortedLines[j][3]), colour, 2, 0);
            }
        }
    }
    
    
    vector<Vec4f> cleanedLines;
    int start = 0, end = 0;

    for(int i = 0; i < k+1; i++){
        Vec4f avgLine = Vec4f(0,0,0,0);
        int count = 0;
        for(int j = 0; j < labels.rows; j++){
            if(labels.at<int>(j) == i){
                avgLine[0] += sortedLines[j][0];
                avgLine[1] += sortedLines[j][1];
                avgLine[2] += sortedLines[j][2];
                avgLine[3] += sortedLines[j][3];
                count++;
            }
        }
        
        avgLine[0] /= count;
        avgLine[1] /= count;
        avgLine[2] /= count;
        avgLine[3] /= count;
        cleanedLines.push_back(avgLine);
    }
    
    for(int i = 0; i < cleanedLines.size(); i++){
        line( cleaned, Point(cleanedLines[i][0], cleanedLines[i][1]), Point(cleanedLines[i][2], cleanedLines[i][3]), Scalar(0,0,255), 2, 0);
    }
    
    imshow("Cleaned", cleaned);

    //findHull(thresh);
    return lines;
}


////////////////////////////////////////////////////////////////////////
///                 MAIN METHOD                                      ///
////////////////////////////////////////////////////////////////////////


int main( int argc, char** argv )
{
    vector<Vec4f> lines;
    //vector<Vec4f> lines = getLines("test0.png");
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
    
    //vector<Vec4f> templateLines { Vec4f(0,0,550,0), Vec4f(0,600,550,600),  Vec4f(550,0,550,600), Vec4f(0,600,550,0), Vec4f(0,0,0,600)};
    vector<Vec4f> templateLines { Vec4f(0,0,1440,0), Vec4f(0,800,1400,800),  Vec4f(1440,0,1440,800), Vec4f(0,0,0,800), Vec4f(430,0,430,800)};
    //vector<Vec4f> templateLines = lines;
    
    for( size_t i = 0; i < templateLines.size(); i++ )
    {
        /*
        templateLines[i][0] += 120;
        templateLines[i][2] += 120;
        
        templateLines[i][1] += 130;
        templateLines[i][3] += 130;
        */
        Vec4f l = templateLines[i];
        line( src2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,255), 2, 0);
    }
    
    Mat matched = imread("test.png", 1);
    
    // Find closest detected line for each template line
    vector<int> alreadyMatched;
    vector<Vec4f> templateMatches(templateLines.size());
    
    vector<Match> matches;
    
    for(int i = 0; i < templateLines.size(); i++)
    {

        Vec4f closest;
        float closestDist = 2000;
        for(int j = 0; j < lines2.size(); j++)
        {
            float dist = lineDistance(templateLines[i], lines2[j]);
            if( getAngle(templateLines[i], lines2[j]) < 70 ){
                matches.push_back( Match(i, j, dist ));
            }
            
            if( dist < closestDist)
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
        //line( matched, Point(closest[0], closest[1]), Point(closest[2], closest[3]), Scalar(255,0,255), 2, 0);
        templateMatches[i] = closest;
        
        // Draw connections between matched line pairs
        
        Vec2f tempMid = Vec2f( ((templateLines[i][0] + templateLines[i][2] )/2) , ((templateLines[i][1] + templateLines[i][3] )/2) );
        Vec2f matchMid = Vec2f( ((closest[0] + closest[2] )/2) , ((closest[1] + closest[3] )/2) );
        //line( src2, Point(tempMid[0], tempMid[1]), Point(matchMid[0], matchMid[1]), Scalar(0,255,100), 2, 0);
    }
    
    sort(matches.begin(), matches.end(), compareMatches);
    vector<Match> bestMatches = getBestMatches(matches);

    //////////////////////////////////*
    /*
    templateLines.clear();
    templateMatches.clear();
    */
    vector<Vec4f> tl;
    vector<Vec4f> tm;
     
    for(int i = 0; i < bestMatches.size(); i++){
        tl.push_back(templateLines[bestMatches[i].l1]);
        tm.push_back(lines2[bestMatches[i].l2]);
    }
    
    //templateLines = tl;
    //templateMatches = tm;
    
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
    
    Mat heck = imread("test.png");
    
    cout << "Total set distance: " << getSetDistance(templateLines, templateMatches) << endl;
    int best = 0;
    for(int i = 0; i < bestMatches.size(); i++){
        best += bestMatches[i].dist;
        
        line( heck, Point(lines2[bestMatches[i].l2][0], lines2[bestMatches[i].l2][1]), Point(lines2[bestMatches[i].l2][2], lines2[bestMatches[i].l2][3]), Scalar(255,0,255), 2, 0);
        cout << i << ",";
    }
    cout << "Best set distance: " << best << endl;
    
             imshow("heck", heck);
    
    
    imshow("TEMPLATE", templateWarped);
    waitKey();
    
    return 0;
}
