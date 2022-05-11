// Algorithm developed by Mez (https://github.com/mez/monocular-visual-odometry)
// Adapted by Matheus Ribeiro (https://github.com/matheusr1321/mono_vo)
// Read README.ME

#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <array>

/**
 ***************** Algorithm Outline
    1. Capture images: It, It+1,
    2. Undistort the above images. (using kitti so this step is done for you!)
    3. Use FAST algorithm to detect features in It, and track those features to It+1. A new detection is triggered if the number of features drop below a certain threshold.
    4. Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
    5. Estimate R,t from the essential matrix that was computed in the previous step.
    6. Take scale information from some external source (like a speedometer), and concatenate the translation vectors, and rotation matrices.
 *************************/

 int first_image = 1; //Define first frame to read
 using namespace std;
 int iteration = 1; // Debug
 using cv::Mat;
 using cv::Point2f;
 using cv::KeyPoint;
 using cv::Size;
 using cv::TermCriteria;

#define MAX_FRAME 15000
#define MIN_NUM_FEAT 2000

// Change this directory to the folder which contain a sequence of frame starting from "000000.png"
const static char* dataset_images_location = "/home/matheus-ubuntu/Desktop/quadrado_try5";

 void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status){
   //this function automatically gets rid of points for which tracking fails

   vector<float> err;
   Size winSize(21,21);
   TermCriteria termcrit(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
   calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

   //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
   int indexCorrection = 0;
   for( int i=0; i<status.size(); i++)
   {
     Point2f pt = points2.at(i- indexCorrection);
     if ((status.at(i) == 0) || (pt.x<0) || (pt.y<0)) {
       if((pt.x<0) || (pt.y<0)) {
         status.at(i) = 0;
       }

       points1.erase (points1.begin() + i - indexCorrection);
       points2.erase (points2.begin() + i - indexCorrection);
       indexCorrection++;
     }
   }
 }

  void featureDetection(Mat img, vector<Point2f> & points){ 
    vector<KeyPoint> keypoints; 
    int fast_threshold = 20;
    bool nonmaxSupression = true; 
    cv::FAST(img, keypoints, fast_threshold, nonmaxSupression);
    KeyPoint::convert(keypoints, points, vector<int>());
  }


int main(int argc, char** argv) {

  bool renderFeatures = false;
  if (argc > 1) {
    renderFeatures = true;
  }

  Mat img_1, img_2;
  Mat R_f, t_f;


  double scale = 1.00;
  char filename1[200];
  char filename2[200];
  sprintf(filename1, "%s/%06d.png", dataset_images_location, first_image); 
  sprintf(filename2, "%s/%06d.png", dataset_images_location, first_image+1); 

  char text[100];
  int fontFace = cv::FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  cv::Point textOrg(10, 50);

  //read the first two frames from the dataset
  Mat img_1_c = cv::imread(filename1);                      
  Mat img_2_c = cv::imread(filename2);

  if ( !img_1_c.data || !img_2_c.data ) {
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

  // we work with grayscale images
  cvtColor(img_1_c, img_1, cv::COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, cv::COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;       
  featureDetection(img_1, points1);
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  //TODO: add a fucntion to load these values directly from KITTI's calib files
  // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
  /*
   * Projection matrix example after rectification: Right Grey scale Camera (3x4)
   *
   *     7.188560000000e+02 0.000000000000e+00 6.071928000000e+02 -3.861448000000e+02
   *     0.000000000000e+00 7.188560000000e+02 1.852157000000e+02  0.000000000000e+00
   *     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00  0.000000000000e+00
   */
  
  //iphone 7 plus calibration info
  double focal = 1220.0899121457219; 
  cv::Point2d pp(639.5, 359.5); //to a 1280x720 camera resolution

  //recovering the pose and the essential matrix
  Mat E, R, R_reference, t, mask;

  E = findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  char filename[100];

  R_f = R.clone();
  R_reference= R.clone(); //Get initial rotation to tests with 10 repetition
  t_f = t.clone();
  clock_t begin = clock();

  cv::namedWindow( "Visual Odometry", cv::WINDOW_NORMAL );// Create a window for display.
  cv::resizeWindow("Visual Odometry", 600,600); //adjust visualization size
  Mat traj = Mat::zeros(600, 1280, CV_8UC3);

  //RGB colors to drawn
  int    color_red=0; 
  int    color_blue=128;
  int    color_green=128;
  
  for(int numFrame=first_image+2; numFrame < MAX_FRAME; numFrame++) {
 /* if(numFrame==1270){ // delete failed repetitions 
      numFrame = 1745;
      continue;
    }    
 */

    sprintf(filename, "%s/%06d.png", dataset_images_location, numFrame);
    Mat currImage_c = cv::imread(filename); 
    if (currImage_c.empty()) continue;
    cvtColor(currImage_c, currImage, cv::COLOR_BGR2GRAY);
    vector<uchar> status;
    featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    //if ( ! E.isContinuous() ) continue; //Error terminate called after throwing an instance of 'cv::Exception' what():  OpenCV(4.5.5-dev) /home/matheus-ubuntu/opencv_build/opencv/modules/core/src/matrix.cpp:1175: error: (-13:Image step is wrong) The matrix is not continuous, thus its number of rows can not be changed in function 'reshape'
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
    Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

    for (int i = 0; i < prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
      prevPts.at<double>(0, i) = prevFeatures.at(i).x;
      prevPts.at<double>(1, i) = prevFeatures.at(i).y;

      currPts.at<double>(0, i) = currFeatures.at(i).x;
      currPts.at<double>(1, i) = currFeatures.at(i).y;
    }

   
   // adjust initial orientation (10 repetitions tests). In this way, all repetitions have the same initial orientation
    /*    if((numFrame>=1 && numFrame<=10) ||(numFrame>=295 && numFrame<=305) || (numFrame>=551 && numFrame<=561) || (numFrame>=784 && numFrame<=794) || (numFrame>=1027 && numFrame<=1060) || (numFrame>=1270 && numFrame<=1280) || (numFrame>=1511 && numFrame<=1521) || (numFrame>=1746 && numFrame<=1756) || (numFrame>=1982 && numFrame<=1992) || (numFrame>=2215 && numFrame<=2225) || (numFrame>=2456 && numFrame<=2466) || (numFrame>=2704 && numFrame<=2714) || (numFrame>=2944 && numFrame<=2954) || (numFrame>=3201 && numFrame<=3211) || (numFrame>=3440 && numFrame<=3450)){
            R_f= R_reference.clone(); 
        }
    */

    t_f = t_f + (R_f * t);
    R_f = R * R_f;

  
    //Make sure we have enough features to track
    if (prevFeatures.size() < MIN_NUM_FEAT) {
      featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
    }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0))*1.0 + 500; //x and y initial coordinates (vizualization)
    int y = int(t_f.at<double>(2))*1.0 +250;

    circle(traj, cv::Point(x, y), 1, CV_RGB(color_red, color_green, color_blue), 4); 
    rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

    if (renderFeatures){
      //Draw features as markers for fun
      for(auto point: currFeatures)
        cv::drawMarker(currImage_c, cv::Point(point.x,point.y), CV_RGB(0,255,0), cv::MARKER_TILTED_CROSS, 2, 1, cv::LINE_AA);
    }

    Mat concated;
    cv::vconcat(currImage_c, traj, concated); //concate matrix (merge to set a unique visualization window) 
    imshow("Visual Odometry", concated);
    cv::waitKey(1);

    // Set a unique color to each repetition
/*    if(numFrame==295){
      t_f=0;  
      color_red=255; //white
      color_blue=255;
      color_green=255;
    }else if(numFrame==551){
          t_f=0;
      color_red=255; //red
      color_blue=0;
      color_green=0;
    }
 */ }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  //let the user press a key to exit.
  cv::waitKey(0);

  return 0;
}