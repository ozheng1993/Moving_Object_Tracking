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
vector<Point2f> mousePoints;
vector<vector<Point2f> > pointsTrackId;
vector<Point2f> countourCenter;
vector<double> countourWidth;
vector<double> countourHeight;
vector<Point2f> newTrackCenter;
vector<bool> carTrakcingStart;
vector<bool> carTrakcingFinished;
vector<bool>  carTrakcingLost;
vector<bool>  carTrakcingRemoved;
vector<double>  carTrakcingWidth;
vector<double>  carTrakcingHeight;
vector<bool>  carTrakcingGetSize;
//vector<int>  carTrakcingId;
int carCount=0;
//vector<Point2f> pointsTrackdeatal;

bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        cout<<"mouse x"<<(float)x<<"mouse y"<<(float)y<<endl;
        mousePoints.push_back(point);
        carTrakcingStart.push_back(true);
        carTrakcingFinished.push_back(false);
        carTrakcingRemoved.push_back(false);
        carTrakcingLost.push_back(false);
        
        
      carTrakcingWidth.push_back(0.0);
    carTrakcingHeight.push_back(0.0);
    carTrakcingGetSize.push_back(false);
        
       // circle( image2, point, 3, Scalar(0,255,0), -1, 8);
        //outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<10<<","<<10<<","<<points[1][i].x<<","<<points[1][i].y<<","<<"tracking"<<",";
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
    pMOG2->setDetectShadows(true);
    pMOG2->setHistory(100);
    pMOG2->setNMixtures(5);
    pMOG2->setBackgroundRatio(0.7);
    pMOG2->setShadowValue(10);
    const int MAX_COUNT = 5000;
    bool needToInit = false;
    bool autoMode= false;
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
    
    //open csv file
    cout<<"filename"<<input<<endl;
    string fileName="./"+input+".csv";
    ofstream outfile;
    outfile.open(fileName);
    outfile<<"frameNUM"<<","<<"time(s)"<<","<< "id"<<","<<"width(m)"<<","<<"height(m)"<<","<<"x"<<","<<"y"<<","<<"speed"<<",";
    outfile<<endl;
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
        
        stringstream ss;
        stringstream st;
        stringstream fps;
        rectangle(frame, cv::Point(10, 2), cv::Point(450,20),
                  cv::Scalar(255,255,255), -1);
        ss << cap.get(CAP_PROP_POS_FRAMES);
        fps << cap.get(CAP_PROP_FPS);
        st << cap.get( CAP_PROP_POS_MSEC);
        string frameNumberString = ss.str();
        string fpsNumberString = fps.str();
        string timeNumberString = st.str();
        double timeFrame=stod(timeNumberString)/1000+40;
        //  string timeNumberString = st.str();
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
        putText(frame, fpsNumberString.c_str(), cv::Point(70, 15),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
        putText(frame, to_string(timeFrame), cv::Point(190, 15),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
        
        
        
       // resize(frame, frame, cv::Size(), 0.5, 0.5);
        frame.copyTo(image);
        frame.copyTo(image2);
        
        // Create a kernel that we will use for accuting/sharpening our image
        Mat kernel = (Mat_<float>(3,3) <<
                      1,  1, 1,
                      1, -8, 1,
                      1,  1, 1); // an approximation of second derivative, a quite strong kernel
        // do the laplacian filtering as it is
        // well, we need to convert everything in something more deeper then CV_8U
        // because the kernel has some negative values,
        // and we can expect in general to have a Laplacian image with negative values
        // BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
        // so the possible negative number will be truncated
        Mat imgLaplacian;
        Mat sharp = image; // copy source image to another temporary one
        filter2D(sharp, imgLaplacian, CV_32F, kernel);
        image.convertTo(sharp, CV_32F);
        Mat imgResult = sharp - imgLaplacian;
        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );
        //imshow( "New Sharped Image", imgResult );
 
        cvtColor(image, gray, COLOR_BGR2GRAY);
        GaussianBlur( gray, blur, Size(3, 3), 5, 5);
        pMOG2->apply(blur, fgMaskMOG2,-1);
//        GaussianBlur( fgMaskMOG2, blur, Size(3, 3), 5, 5);
//        pMOG2->apply(fgMaskMOG2, fgMaskMOG2,-1);
//        //imshow("fgMaskMOG21", fgMaskMOG2);
//        //threshold(blur, blur, .1, 1., CV_THRESH_BINARY);
//
//
//        //imshow("blur", blur);
//
//        Mat open =Mat::ones(Size(5,5),CV_8U);
//        Mat close =Mat::ones(Size(3,3),CV_8U);
//    morphologyEx( fgMaskMOG2, fgMaskMOG2, MORPH_OPEN, open );
//     //morphologyEx( blur, blur, MORPH_CLOSE, open );
//        distanceTransform (fgMaskMOG2, fgMaskMOG2, CV_DIST_L2, 0);
//
//        normalize(fgMaskMOG2, fgMaskMOG2, 0.1, 1., NORM_MINMAX);
//        //imshow("blur2", fgMaskMOG2);
//        threshold(fgMaskMOG2, fgMaskMOG2, .2, 1., CV_THRESH_BINARY);
//
//        fgMaskMOG2.convertTo(fgMaskMOG2, CV_8U);

        //imshow("blur22", fgMaskMOG2);
      // blur.convertTo(blur, CV_8U);
    // Canny( fgMaskMOG2, fgMaskMOG2, 50, 150, 7);
        
        
       // imshow("mog Demo", fgMaskMOG2);


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
            double a=contourArea( contours[i],false);
            // cout<<a<<endl;
              if(a>50)
                {
                    int contourX=0;
                    int contourY=0;
                    double contourW=0.0;
                    double contourH=0.0;
                    contourX=boundRect[i].tl().x+(boundRect[i].br().x-boundRect[i].tl().x)/2;
                    contourY=boundRect[i].tl().y+(boundRect[i].br().y-boundRect[i].tl().y)/2;
                    
                    contourW=boundRect[i].br().x-boundRect[i].tl().x;
                    contourH=boundRect[i].br().y-boundRect[i].tl().y;
                    
                    countourWidth.push_back(contourW);
                    countourHeight.push_back(contourH);
                    Point2f temPoint;
                    temPoint=Point2f(contourX, contourY);
                    countourCenter.push_back(temPoint);
//                    addRemovePt = true;
                    if( !points[1].empty() )
                    {

                        for( int j=0; j < points[1].size(); j++ )
                        {
                           // cout<<"search for car size"<<endl;
                            if( points[1][j].x<=boundRect[i].br().x&&
                                points[1][j].y<=boundRect[i].br().y&&
                                points[1][j].x>=boundRect[i].tl().x&&
                                points[1][j].y>=boundRect[i].tl().y&&
                                carTrakcingGetSize[j]==false)
                            {
                                cout<<"find car size"<<contourW<<"|"<<contourH<<endl;
                                carTrakcingWidth[j]=contourW;
                                carTrakcingHeight[j]=contourH;
                                carTrakcingGetSize[j]=true;
                                continue;
                            }
                        }
                    }
//
//                        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//                        {
//                            vector<Point2f> tmp;
//                            tmp.push_back(temPoint);
//                            //cornerSubPix( newGray, tmp, winSize, Size(-1,-1), termcrit);
                      //  points[1].push_back(temPoint);
//                            addRemovePt = false;
//                        }
//
//
//
//
//
//
//                    }
//
                    
                    // string carDinit = "CD init Dfor car: "+to_string(i);
                    drawContours( image, contours_poly, i, Scalar(255,0,255), 2, 8, vector<Vec4i>(), 0, Point() );
                    rectangle( image, boundRect[i].tl(), boundRect[i].br(),Scalar(255,0,255), -1, 8, 0);
                    rectangle( image2, boundRect[i].tl(), boundRect[i].br(),Scalar(255,0,255), 1, 8, 0);
                }
        }

        Mat newGray;
        cvtColor(image, hsv, COLOR_BGR2HSV);
        cvtColor(hsv, newGray, COLOR_BGR2GRAY);
        
        if( nightMode )
            image2 = Scalar::all(0);
        
        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(newGray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0, 0.04);
            //cornerSubPix(newGray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {
  
//            for( int i=0; i < points[0].size(); i++ )
//            {
//                putText(image2, to_string(i), points[0][i], FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//            }
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                newGray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, newGray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            //int counter=0;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                //cout<<points[1]<<endl;
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 10 &&carTrakcingRemoved[i]!=true)
                    {
                        carTrakcingRemoved[i]=true;
                        carTrakcingFinished[i]=true;
                        addRemovePt = false;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<"removed"<<",";
                        continue;
                        
                    }
                }
                if( !status[i] )
                {
                    if(points[1][i].x<10 || points[1][i].x>1700)
                    {
                        carTrakcingFinished[i]=true;
                        carTrakcingStart[i]=false;
                        cout<<"car "<<i<<"tracking finished"<<endl;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<points[1][i].x<<","<<points[1][i].y<<","<<"finished"<<",";
                        continue;
                    }
                    else if(carTrakcingLost[i]!=true)
                    {
                        carTrakcingFinished[i]=true;
                        carTrakcingStart[i]=false;
                        carTrakcingLost[i]=true;
                        cout<<"car "<<i<<"lost"<<endl;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<"lost"<<",";
                        continue;
                    }
                  
                   
                }
                
                //points[1][k++] = points[1][i];
                if(carTrakcingFinished[i]==false)
                {
                circle( image2, points[1][i], 3, Scalar(0,255,0), -1, 8);
                putText(image2, to_string(i), points[0][i], FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
                //outfile<<"frameNUM"<<","<<"time(s)"<<","<< "id"<<","<<"width(m)"<<","<<"height(m)"<<","<<"x"<<","<<"y"<<",";
                    outfile<<endl;
                    outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<"tracking"<<",";
                }
               // counter++;
            }
           //points[1].resize(k);
        }
    
        
        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            //cornerSubPix( newGray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }
        
        needToInit = false;
        //record start point
//        if(!mousePoints.empty())
//        {
//            for(int i=0; i < mousePoints.size(); i++ )
//            {
//                circle( image2, mousePoints[i], 3, Scalar(255,0,0), -1, 8);
//                putText(image2, to_string(i), mousePoints[i], FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//
//            }
//        }
//check couter
        
        
        
        
        if( !points[0].empty() &&autoMode)
        {
            
            
            for( int k=0; k < countourCenter.size(); k++ )
            {
             bool newPoint=true;
                for( int i=0; i < points[0].size(); i++ )
                {
                    //check contour center
                    //Point2f temTrackPoint;
                   
                      // temTrackPoint=countourCenter[k];
                    
                    
                    
//                 if(countourCenter[k].x>400&&countourCenter[k].x<1600 )
//              {
//                       if( norm(countourCenter[k] - points[1][i]) <= 50 )
//                       {
//                           newPoint=false;
//
//
//                       }
//
//                    }
//
                if(countourCenter[k].x>100 )
                    {
                         newPoint=false;
                        break;
                    }


                        if( norm(countourCenter[k] - points[1][i]) <= 20 )
                        {
                            newPoint=false;
                            break;

                        }

                    
//                    if( norm(countourCenter[k] - points[1][i]) <= 50 )
//                                               {
//                                                   newPoint=false;
//
//                                                   break;
//                                               }
//
                   }
                if(newPoint==true)
                {
                    
                    addRemovePt = true;
                    
                    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
                    {
                        vector<Point2f> tmp;
                        tmp.push_back(countourCenter[k]);
                        carTrakcingStart.push_back(true);
                        //cornerSubPix( newGray, tmp, winSize, Size(-1,-1), termcrit);
                        points[1].push_back(tmp[0]);
                        carTrakcingStart.push_back(true);
                        carTrakcingFinished.push_back(false);
                        carTrakcingRemoved.push_back(false);
                        carTrakcingLost.push_back(false);
                        carTrakcingWidth.push_back(0.0);
                        carTrakcingHeight.push_back(0.0);
                        carTrakcingGetSize.push_back(false);
                        addRemovePt = false;
                    }
                    
                    
                    
                   // newTrackCenter.push_back(countourCenter[k]);
                    
                    //circle( image2, countourCenter[k], 3, Scalar(255,0,0), -1, 8);
                   // putText(image2, to_string(i), mousePoints[i], FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
                }
                
                
                
            }
        }
        
        countourCenter.clear();
        imshow("LK Demo", image2);
        
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
               carTrakcingStart.clear();
                carTrakcingFinished.clear();
                carTrakcingLost.clear();
                 carTrakcingRemoved.clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
            case 'a':
                autoMode = !autoMode;
                break;
        }
        std::swap(points[1], points[0]);
        cv::swap(prevGray, newGray);
    }
    
    return 0;
}
