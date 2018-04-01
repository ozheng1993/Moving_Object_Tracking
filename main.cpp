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
bool selectRot=false;
bool selectScale=false;
bool tracking=false;
bool drawLIne=false;
double pixeToMeter=0;
double realPixel=0;
double realMetter=0;
vector<float> preX;
vector<float> preY;
vector<float> lastX;
vector<float> lastY;
vector<float> speed;
vector<float> speedX;
vector<float> speedY;
vector<float> speedCounter;
vector<float> speedPreAvg;
vector<float> speedAvg;
vector<float> lostcount;
vector<bool> finish;
vector<bool> start;
vector<bool> lost;
Point2f point;
vector<Point2f> mousePoints;
vector<vector<Point2f> > pointsTrackId;
vector<Point2f> countourCenter;
vector<double> countourWidth;
vector<double> countourHeight;
vector<Point2f> carTrackCenter;
vector<bool> carTrakcingStart;
vector<bool> carTrakcingFinished;
vector<bool>  carTrakcingLost;
vector<bool>  carTrakcingRemoved;
vector<double>  carTrakcingWidth;
vector<double>  carTrakcingHeight;
vector<bool>  carTrakcingGetSize;
vector<float>  carTrakcingSpeed;
Mat frameMat;
int skipFrame=6;
//vector<int>  carTrakcingId;
int carCount=0;
//vector<Point2f> pointsTrackdeatal;

bool addRemovePt = false;
Mat Roi;

vector<Point>  roiPoint;
vector<Point> scalePoint;
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
        preX.push_back(0);
        preY.push_back(0);
        lastX.push_back(0);
        lastY.push_back(0);
        speed.push_back(0);
        speedX.push_back(0);
        speedY.push_back(0);
        carTrackCenter.push_back(point);
        carTrakcingSpeed.push_back(0);
      carTrakcingWidth.push_back(0.0);
    carTrakcingHeight.push_back(0.0);
    carTrakcingGetSize.push_back(false);
        
       // circle( image2, point, 3, Scalar(0,255,0), -1, 8);
        //outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<10<<","<<10<<","<<points[1][i].x<<","<<points[1][i].y<<","<<"tracking"<<",";
        addRemovePt = true;
     
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
          //  cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        if(!selectRot)
        {
            roiPoint.push_back( Point(x,y));
            cout<<roiPoint[roiPoint.size()-1]<<endl;
        }
        
        
     
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
     
        //  cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        if(!selectScale)
        {
        scalePoint.push_back( Point(x,y));
        cout<<scalePoint[scalePoint.size()-1]<<endl;
        }
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    }
}
double drwaLine(Mat frame, int x1, int y1, int x2, int y2)
{
    cv::line(frame,
             cv::Point(x1,y1)
             ,
             cv::Point(x2,y2),
             cv::Scalar(255,0,0),
             2,
             LINE_8
             );
    double lineDist=sqrt(pow((x1-x2),2)-pow((y1-y2),2));
    cout<<"linedistance"<<lineDist<<endl;
    return lineDist;
    
}

Mat resizeWindow(Mat frame)
{
    Roi=frame;
    Mat src=Roi;
    Mat dst=Roi;
    int new_w=0;
    int new_h=0;
    new_w=src.cols;
    new_h=src.rows;
    Rect rectROI(0,0,new_w,new_h);
    Mat mask(new_h, new_w, CV_8UC1, cv::Scalar(0));
    Point P1(0,210);
    Point P1_0(300,175);
    Point P1_1(600,145);
    Point P1_2(1300,145);
    Point P2(1920,160);
    Point P3(1920,250);
    Point P3_1(1300,220);
    Point P3_2(900,250);
    Point P3_3(600,250);
    Point P3_4(300,300);
    Point P4(0,350);
    vector< vector<Point> >  co_ordinates;
    co_ordinates.push_back(vector<Point>());
    for(int i=0;i<roiPoint.size();i++)
    {
           co_ordinates[0].push_back(roiPoint[i]);
    }
 

    drawContours( mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
    Mat srcROI=src(rectROI);
    Mat dstROI=dst(rectROI);
    Mat dst1;
    srcROI.copyTo(dst1,mask);
    return dst1;
}
void processVideo(char* videoFilename);
int main( int argc, char** argv )
{
    //VideoCapture cap;
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
    
    
    
    if(argc<2){
        cout<<
        " Usage: tracker <video_name>\n"
        " examples:\n"
        " example_tracking_kcf Bolt/img/%04d.jpg\n"
        " example_tracking_kcf faceocc2.webm\n"
        << endl;
        return 0;
    }
    std::string video = argv[1];
    VideoCapture cap(video);
    stringstream fps;
    
     fps << cap.get(CAP_PROP_FPS);
    
    
    //open csv file
    cout<<"filename"<<video<<endl;
    string fileName="./"+video+".csv";
    ofstream outfile;
    outfile.open(fileName);
    outfile<<"this video is "<<fps.str()<<"fps"<<"speed cal skip every"<<skipFrame;
    outfile<<endl;
    outfile<<"frameNUM"<<","<<"time(s)"<<","<< "id"<<","<<"width(m)"<<","<<"height(m)"<<","<<"x"<<","<<"y"<<","<<"speed"<<","<<"status"<<",";
    outfile<<endl;
     Mat gray, prevGray, image, frame,blur,image2;
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, NULL );
       //  cout<<"please select ROI"<<endl;

  
   
    vector<Point2f> points[2];
    vector<int> id;


//    while(!selectRot)
//    {
//        cout<<roiPoint[0].size()<<endl;
//        rectangle(frameMat, cv::Point(10, 2), cv::Point(650,20),
//                  cv::Scalar(255,255,255), -1);
//        string displayInfor="please select ROI by right click mouse,esc to cancle,entry to finish";
//        putText(frameMat, displayInfor.c_str(), cv::Point(15, 15),
//                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//
//
//
//        if(waitKey(1)==27)
//        {
//            roiPoint[0].pop_back();
//            cv::circle(frameMat,
//                       cv::Point(roiPoint[0][roiPoint[0].size()].x,roiPoint[0][roiPoint[0].size()].y),
//                       3,
//                       cv::Scalar(0,255,0),
//                       -1,
//                       LINE_8
//                       );
//
//            cout<<"esc pressed"<<roiPoint[0][0].x<<endl;
//
//        }
//        else if(waitKey(1)==13)
//        {
//            cout<<"roi entry key predded"<<endl;
//            frame= resizeWindow(frame);
//            selectRot=true;
//            // close the window
//            //DestroyWindow("My Window");
//            //frameMat.release();
//            break;
//        }
//
//        imshow("LK Demo", frameMat);
//    }

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    bool puse=false;
    for(;;)
    {
        cap >> frame;
        
        rectangle(frame, cv::Point(10, 2), cv::Point(650,20),
                  cv::Scalar(255,255,255), -1);
        frameMat=frame;
        if( frame.empty() )
            break;
        if(waitKey(1)==27)
        {
            puse=!puse;
        }
        while(puse)
        {
//            if(!mousePoints.empty())
//            {
//                for(int i =0;i<mousePoints.size();i++)
//                {
//                 circle( frame, mousePoints[i], 3, Scalar(0,255,0), -1, 8);
//                }
//            }
            if(waitKey(1)==27)
            {
                puse=!puse;
                break;
            }
            
        }
        
//        while(!selectScale)
//        {
//
//            frameMat=frameMat;
//            rectangle(frameMat, cv::Point(10, 2), cv::Point(650,20),
//                      cv::Scalar(255,255,255), -1);
//
//            string displayInfor="please select scale by mid click mouse,esc to cancle,entry to finish";
//            putText(frameMat, displayInfor.c_str(), cv::Point(15, 15),
//                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//
//
//            if(waitKey(1)==27)
//            {
//                scalePoint[0].pop_back();
//                cv::circle(frame,
//                           cv::Point(scalePoint[0][scalePoint[0].size()].x,scalePoint[0][scalePoint[0].size()].y),
//                           3,
//                           cv::Scalar(0,255,0),
//                           -1,
//                           LINE_8
//                           );
//
//                cout<<"esc pressed"<<scalePoint[0][0].x<<endl;
//
//            }
//            else if(waitKey(1)==13)
//            {
//                cout<<"selectScale entry key predded"<<endl;
//                double line1x1=scalePoint[0][0].x;
//                double line1y1=scalePoint[0][0].y;
//                double line1x2=scalePoint[0][1].x;
//                double line1y2=scalePoint[0][1].y;
//
//                cv::line(frameMat,
//                         cv::Point(line1x1,line1y1)
//                         ,
//                         cv::Point(line1x2,line1y2),
//                         cv::Scalar(255,0,0),
//                         2,
//                         LINE_8
//                         );
//
//                double xdiff=line1x1-line1x2;
//                double ydiff=line1y1-line1y2;
//                double ydiff2=ydiff*ydiff;
//                double xdiff2=xdiff*xdiff;
//
//                double lineDist=sqrt(ydiff2+xdiff2);
//                cout<<"xdiff2"<<xdiff2<<endl;
//                cout<<"ydiff2"<<ydiff2<<endl;
//                cout<<"linedistance"<<lineDist<<endl;
//                //cout<<"reallMeter"<<reallMeter<<endl;
//                // pixelToMeterfinal=lineDist/reallMeter;
//                realPixel=lineDist;
//                selectScale=true;
//                // close the window
//                //DestroyWindow("My Window");
//                // frameMat.release();
//                break;
//            }
//
//
//            imshow("LK Demo", frameMat);
//
//        }

        char d = (char)waitKey(10);
   
        switch( d )
        {
            case 'u':
                if(roiPoint.size()>0)
                {
                scalePoint.pop_back();
                }
                break;
            case 'i':
                if(scalePoint.size()>0)
                {
               scalePoint.pop_back();
                }
                break;
            case 'o':
                 selectScale=true;
                break;
            case 'p':
                 selectRot=true;
                break;
        }
        if(roiPoint.size()>0&&!selectRot)
        {
            for(int i=0;i<roiPoint.size();i++)
            {
                circle( frame, roiPoint[i], 3, Scalar(255,125,255), -1, 8);
            }
        }
        if(scalePoint.size()>0&&!selectScale)
        {
            for(int i=0;i<scalePoint.size();i++)
            {
                circle( frame, scalePoint[i], 3, Scalar(255,125,0), -1, 8);
            }
        }
        if(selectScale&&drawLIne==false)
        {
            double line1x1=scalePoint[0].x;
                            double line1y1=scalePoint[0].y;
                            double line1x2=scalePoint[1].x;
                            double line1y2=scalePoint[1].y;
            
                            cv::line(frameMat,
                                     cv::Point(line1x1,line1y1)
                                     ,
                                     cv::Point(line1x2,line1y2),
                                     cv::Scalar(255,0,0),
                                     2,
                                     LINE_8
                                     );
            
                            double xdiff=line1x1-line1x2;
                            double ydiff=line1y1-line1y2;
                            double ydiff2=ydiff*ydiff;
                            double xdiff2=xdiff*xdiff;
            
                            double lineDist=sqrt(ydiff2+xdiff2);
                            cout<<"xdiff2"<<xdiff2<<endl;
                            cout<<"ydiff2"<<ydiff2<<endl;
                            cout<<"linedistance"<<lineDist<<endl;
                            //cout<<"reallMeter"<<reallMeter<<endl;
                            // pixelToMeterfinal=lineDist/reallMeter;
                            realPixel=lineDist;
            drawLIne=true;
            outfile<<endl;
            outfile<<"line length:"<<lineDist;
            
        }
        
        if(selectRot)
        {
             frame= resizeWindow(frame);
        }

       // cout<<roiPoint.size()<<endl;
        
        
        
        
        
        
        
        
        stringstream ss;
        stringstream st;
        
        rectangle(frame, cv::Point(10, 2), cv::Point(450,20),
                  cv::Scalar(255,255,255), -1);
        ss << cap.get(CAP_PROP_POS_FRAMES);
      
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
            
              if(a>100&&a<3000)
              {
                  // cout<<a<<endl;
                    int contourX=0;
                    int contourY=0;
                    double contourW=0.0;
                    double contourH=0.0;
                  
                  
                  double x1;
                  double y1;
                  double x2;
                  double y2;
                  
                  contourW=boundRect[i].br().x-boundRect[i].tl().x;
                  contourH=boundRect[i].br().y-boundRect[i].tl().y;
                  
                  if(contourW<35)
                  {
                      x1= boundRect[i].tl().x+(boundRect[i].br().x-boundRect[i].tl().x)/5;
                      y1= boundRect[i].tl().y+(boundRect[i].br().y-boundRect[i].tl().y)/8;
                      x2= boundRect[i].br().x-(boundRect[i].br().x-boundRect[i].tl().x)/5;
                       y2= boundRect[i].br().y-(boundRect[i].br().y-boundRect[i].tl().y)/2;
                  }
                  else
                  {
                      x1= boundRect[i].tl().x+(boundRect[i].br().x-boundRect[i].tl().x)/5;
                      y1= boundRect[i].tl().y+(boundRect[i].br().y-boundRect[i].tl().y)/8;
                      x2= boundRect[i].br().x-(boundRect[i].br().x-boundRect[i].tl().x)/5;
                      y2= boundRect[i].br().y-(boundRect[i].br().y-boundRect[i].tl().y)/2;
                  }
                  
                  
                  contourX=x1+(x2-x1)/2;
                  contourY=y1+(y2-y1)/2;
                  
             
                  
                  
                  
                  
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
                  rectangle( image, cv::Point(x1,y1),cv::Point(x2,y2),Scalar(255,0,255), -1, 8, 0);
                    rectangle( image2, boundRect[i].tl(), boundRect[i].br(),Scalar(255,0,255), 1, 8, 0);
                }
        }

        
        //bitwise_or(image,fgMaskMOG2,image);
        
        Mat newGray;
        cvtColor(image, hsv, COLOR_BGR2HSV);
        cvtColor(hsv, newGray, COLOR_BGR2GRAY);
        //Mat res;
       // bitwise_or(newGray, newGray, res);      imshow("OR", res);
        
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
                
//                if(stoi(frameNumberString)%skipFrame==0)
//                {
//                    //cout<<"tracker speed: "<<i<<"i"<<endl;
//                    if(lastX[i]==0&&lastY[i]==0)
//                    {
//                        //cout<<"init speed: "<<i<<"i"<<endl;
//                        // cout<<"first time : "<<i<<endl;
//                        lastX[i]=points[1][i].x;
//                        lastY[i]=points[1][i].y;
//                        speed[i]=0;
//                    }
//                    else
//                    {
//
//                        double tmp=0;
//                        //cout<<"cal speed: "<<i<<"i"<<endl;
//                        //speedCounter[i]+=1;
//                        preX[i]=points[1][i].x;
//                        preY[i]=points[1][i].y;
//                        speedX[i]=preX[i]-lastX[i];
//                        speedY[i]=preY[i]-lastY[i];
//                        lastX[i]=preX[i];
//                        lastY[i]=preY[i];
//                        tmp=sqrt((speedX[i]*speedX[i])-(speedY[i]*speedY[i]));
////                        if(abs(tmp- carTrakcingSpeed[i])<5&&tmp>0)
////                        {
//                            carTrakcingSpeed[i]=tmp;
////                        }
//
//                    }
//
//                }

                //cout<<points[1]<<endl;
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 10 &&carTrakcingRemoved[i]!=true)
                    {
                        carTrakcingRemoved[i]=true;
                        carTrakcingFinished[i]=true;
                        addRemovePt = false;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<carTrakcingSpeed[i]<<","<<"removed"<<",";
                        continue;
                        
                    }
                }
                if( !status[i] )
                {
                    if((points[1][i].x<10 || points[1][i].x>1700)&&carTrakcingFinished[i]!=true)
                    {
                        carTrakcingFinished[i]=true;
                        carTrakcingStart[i]=false;
                        cout<<"car "<<i<<"tracking finished"<<endl;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<points[1][i].x<<","<<points[1][i].y<<","<<carTrakcingSpeed[i]<<","<<"finished"<<",";
                        continue;
                    }
                    else if(carTrakcingLost[i]!=true)
                    {
                        carTrakcingFinished[i]=true;
                        carTrakcingStart[i]=false;
                        carTrakcingLost[i]=true;
                        cout<<"car "<<i<<"lost"<<endl;
                        outfile<<endl;
                        outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<carTrakcingSpeed[i]<<","<<"lost"<<",";
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
                    outfile<<frameNumberString<<","<<to_string(timeFrame)<<"," << i<<","<<carTrakcingWidth[i]<<","<<carTrakcingHeight[i]<<","<<points[1][i].x<<","<<points[1][i].y<<","<<carTrakcingSpeed[i]<<","<<"tracking"<<",";
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
                    //check contour cente
//
                if(countourCenter[k].x>300 )
                    {
                         newPoint=false;
                        break;
                    }
                    
                    if(countourCenter[k].x<80 )
                    {
                        newPoint=false;
                        break;
                    }
                        if( norm(countourCenter[k] - points[1][i]) <= 20 )
                        {
                            newPoint=false;
                            break;

                        }
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
                        preX.push_back(0);
                        preY.push_back(0);
                        lastX.push_back(0);
                        lastY.push_back(0);
                        speed.push_back(0);
                        speedX.push_back(0);
                        speedY.push_back(0);
                        carTrakcingSpeed.push_back(0);
                         carTrackCenter.push_back(countourCenter[k]);
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
         imshow("LK Dem1o", image);
        char c = (char)waitKey(10);
//        if( c == 27 )
//            break;
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
