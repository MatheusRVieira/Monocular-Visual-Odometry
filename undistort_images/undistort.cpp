// convert ground truth to IMU? https://stackoverflow.com/questions/60639665/visual-odometry-kitti-dataset
// this also can be useful: https://stackoverflow.com/questions/55756530/format-of-kitti-poses-dataset-poses-and-how-re-create-using-imu
//rename files https://ostechnix.com/how-to-rename-multiple-files-at-once-in-linux/
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

/**
 ***************** Algorithm Outline
    1. Capture images: It, It+1,
    2. Undistort the above images. (using kitti so this step is done for you!)
    3. Use FAST algorithm to detect features in It, and track those features to It+1. A new detection is triggered if the number of features drop below a certain threshold.
    4. Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
    5. Estimate R,t from the essential matrix that was computed in the previous step.
    6. Take scale information from some external source (like a speedometer), and concatenate the translation vectors, and rotation matrices.
 *************************/

 using namespace std;
 using cv::Mat;
 using cv::Point2f;
 using cv::KeyPoint;
 using cv::Size;
 using cv::TermCriteria;

#define MAX_FRAME 7000
#define MIN_NUM_FEAT 2000

//const static char* dataset_images_location = "/home/matheus-ubuntu/Desktop/sequences/00/image_1";
 const static char* dataset_images_location = "/home/matheus-ubuntu/Desktop/oficial3";
const static char* dataset_poses_location = "/home/matheus-ubuntu/Desktop/dataset/poses/01.txt";


int main(int argc, char** argv) {

  float f = 1.2200899121457219e+03;
  float cx = 6.3950000000000000e+02;
  float cy = 3.5950000000000000e+02;
  Mat cameraMatrix = Mat(3, 3, CV_32FC1);
  cameraMatrix.ptr<float>(0)[0]=f;
  cameraMatrix.ptr<float>(0)[1]=0;
  cameraMatrix.ptr<float>(0)[2]=cx;
  cameraMatrix.ptr<float>(1)[0]=0;
  cameraMatrix.ptr<float>(1)[1]=f;
  cameraMatrix.ptr<float>(1)[2]=cy;
  cameraMatrix.ptr<float>(2)[0]=0;
  cameraMatrix.ptr<float>(2)[1]=0;
  cameraMatrix.ptr<float>(2)[2]=1;

  Mat distCoeffs = Mat(1, 5, CV_32FC1);
  distCoeffs.ptr<float>(0)[0]=2.5878587994055857e-01;
  distCoeffs.ptr<float>(0)[1]=-1.4822072855187385e+00;
  distCoeffs.ptr<float>(0)[2]=0;
  distCoeffs.ptr<float>(0)[3]=0;
  distCoeffs.ptr<float>(0)[4]=7.9401453905466135e-01;
    
  double scale = 1.00;
  char filename1[200];
  char filename2[200];

  // aqui começa de fato o loop
  for(int numFrame=1; numFrame < 10; numFrame++) {
    //cout<<numFrame<<endl; 
    sprintf(filename1, "%s/%06d.png", dataset_images_location, numFrame); //filename recebe o endereço do frame 2 até o frame 2000
    // repete-se o mesmo processo anterior
    cout<<cameraMatrix<<endl;
    cout<<distCoeffs<<endl;
    Mat currImage = cv::imread(filename1);
    Mat corrImage = currImage.clone();
    undistort(currImage, corrImage, cameraMatrix, distCoeffs); 
    imshow("image view",corrImage);
    int k = cv::waitKey(0);
    //cvtColor(currImage_c, currImage, cv::COLOR_BGR2GRAY);
    //cv::imwrite(filename, currImage);
    //cout<< "com cor: "<<currImage_c.size()<<" gray: "<<currImage.size()<<endl;
    }

  return 0;
}