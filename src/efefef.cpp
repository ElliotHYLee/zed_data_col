#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <iostream>
#include <fstream>
using namespace sl;
using namespace std;

cv::Mat slMat2cvMat(Mat& input);
int writeCamParam(CalibrationParameters cp)
{
  ofstream myfile;
  myfile.open ("camparam.txt");
  myfile << "fx_left " << cp.left_cam.fx <<endl;
  myfile << "fy_left " << cp.left_cam.fy<<endl;
  myfile << "fx_right " << cp.right_cam.fx<<endl;
  myfile << "fy_right " << cp.right_cam.fy<<endl;
  myfile << "cx_left " << cp.left_cam.cx<<endl;
  myfile << "cy_left " << cp.left_cam.cy<<endl;
  myfile << "cx_right " << cp.right_cam.cx<<endl;
  myfile << "cy_right " << cp.right_cam.cy<<endl;
  myfile << "cam_tx " << cp.T[0]<<endl;
  myfile << "rot_x " << cp.R[0]<<endl;
  myfile << "rot_y " << cp.R[1]<<endl;
  myfile << "rot_z " << cp.R[2]<<endl;
  myfile.close();
}

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.sdk_verbose = false; // Disable verbose mode
    initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters.coordinate_units = UNIT_METER;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    
    // Open the camera
    ERROR_CODE err = zed.open();
    if (err != SUCCESS) exit(-1);

    CalibrationParameters calibration_params = zed.getCameraInformation().calibration_parameters;
    writeCamParam(calibration_params);

    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
       if (zed.grab(runtime_parameters) == SUCCESS) {

           // Retrieve the left image, depth image in half-resolution
           zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
           zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);

           // Retrieve the RGBA point cloud in half-resolution
           // To learn how to manipulate and display point clouds, see Depth Sensing sample
           //zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);

           // Display image and depth using cv:Mat which share sl:Mat data
           cv::imshow("Image", image_ocv);
           cv::imshow("Depth", depth_image_ocv);

           // Handle key event
           key = cv::waitKey(10);
           //processKeyEvent(zed, key);
        }
    }



    // Close the camera
    zed.close();
    return 0;
}

cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
