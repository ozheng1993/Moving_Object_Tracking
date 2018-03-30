#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
//ou zheng
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <time.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/video.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/background_segm.hpp"
#include <math.h>       /* pow */
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
using namespace cv;
using namespace std;
Mat fgMaskMOG2;
Mat hsv;
Ptr<BackgroundSubtractorMOG2> pMOG2; //MOG2 Background subtractor
static void help()
{
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
    "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
    "\tESC - quit the program\n"
    "\tr - auto-initialize tracking\n"
    "\tc - delete all the points\n"
    "\tn - switch the \"night\" mode on/off\n"
    "To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}

int main( int argc, char** argv )
{
    VideoCapture cap;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    //MOG2 approach
    pMOG2 = createBackgroundSubtractorMOG2();
    //pMOG2->setDetectShadows(true);
    pMOG2->setHistory(100);
    //pMOG2->setNMixtures(5);
    pMOG2->setBackgroundRatio(0.7);
    const int MAX_COUNT = 5000;
    bool needToInit = false;
    bool nightMode = false;
    
    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");
    
    if( input.size() == 1 && isdigit(input[0]) )
        cap.open(input[0] - '0');
    else
        cap.open(input);
    
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    
    Mat gray, prevGray, image, frame,blur,image2;
    vector<Point2f> points[2];
    vector<int> id;
    
    for(;;)
    {
        cap >> frame;
        if( frame.empty() )
            break;
       // resize(frame, frame, cv::Size(), 0.5, 0.5);
        frame.copyTo(image);
        frame.copyTo(image2);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        GaussianBlur( gray, blur, Size(5, 5), 5, 5);
        pMOG2->apply(blur, fgMaskMOG2,-1);
        
        
        Mat open =Mat::ones(Size(3,3),CV_8U);
        Mat close =Mat::ones(Size(3,3),CV_8U);
        morphologyEx( fgMaskMOG2, fgMaskMOG2, MORPH_OPEN, open );
        morphologyEx( fgMaskMOG2, fgMaskMOG2, MORPH_CLOSE, open );
        Canny( fgMaskMOG2, fgMaskMOG2, 50, 150, 3);
      
        
        
        
        
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        /// Find contours
        findContours( fgMaskMOG2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0) );
        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
           
            
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            // boundRect2[i] = boundingRect( Mat(contours_poly[i]) );
            minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }
        /// Draw polygonal contour + bonding rects + circles
        Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            
//            float cx = boundRect[i].x+boundRect[i].width/2;
//            float cy = boundRect[i].y+boundRect[i].height/2;
//            point = Point2f(cx, cy);
//            vector<Point2f> tmp;
//            tmp.push_back(point);
//            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
//            points[1].push_back(tmp[0]);
//            addRemovePt = false;
            
                    // string carDinit = "CD init Dfor car: "+to_string(i);
                    drawContours( image, contours_poly, i, Scalar(255,0,255), 2, 8, vector<Vec4i>(), 0, Point() );
                    rectangle( image, boundRect[i].tl(), boundRect[i].br(),Scalar(255,0,255), -1, 8, 0);
//            drawContours( fgMaskMOG2, contours_poly, i, Scalar(255,0,255), 2, 8, vector<Vec4i>(), 0, Point() );
//            rectangle( fgMaskMOG2, boundRect[i].tl(), boundRect[i].br(),Scalar(255,255,255), -1, 8, 0);

        }
  
//        //gray=fgMaskMOG2;
//        image.copyTo(image2);
        
        
        cvtColor(image, hsv, COLOR_BGR2HSV);
        cvtColor(hsv, gray, COLOR_BGR2GRAY);
       //
       // imshow("mog Demo", fgMaskMOG2);
//      imshow("gary", gray);
//        imshow("gar2y", hsv);
        
        if( nightMode )
            image = Scalar::all(0);
        
        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {
            
            
//            for( int i=0; i < points[0].size(); i++ )
//            {
//                putText(image, to_string(i), points[0][i], FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//            }
//            
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            //int counter=0;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                //cout<<points[1]<<endl;
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }
                
                if( !status[i] )
                    continue;
                
                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
               // counter++;
                
                
            }
            points[1].resize(k);
        }
    
        
        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }
        
        needToInit = false;
        imshow("LK Demo", image);
        
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c )
        {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
    
    return 0;
}
