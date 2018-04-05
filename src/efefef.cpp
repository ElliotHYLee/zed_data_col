#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <iostream>
#include <fstream>
using namespace sl;
using namespace std;
void transformPose(sl::Transform &pose, float tx);
cv::Mat slMat2cvMat(Mat& input);
int writeCamParam(CalibrationParameters cp)
{
  ofstream myfile;
  myfile.open ("../Data/camparam.txt");
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
    Pose camera_pose;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.sdk_verbose = false; // Disable verbose mode
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;


    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) exit(-1);

    // Set positional tracking parameters
    TrackingParameters trackingParameters;
    trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.enable_spatial_memory = true;

    float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;
    // Start motion tracking
    zed.enableTracking(trackingParameters);

    CalibrationParameters calibration_params = zed.getCameraInformation().calibration_parameters;
    writeCamParam(calibration_params);

    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    Mat image_zed_left(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv_left = slMat2cvMat(image_zed_left);
    Mat image_zed_right(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv_right = slMat2cvMat(image_zed_right);

    Mat depth_image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

    ofstream myfile;
    myfile.open ("../Data/data.txt");

    // Loop until 'q' is pressed
    int index = 0;
    char key = ' ';
    long prevTime = 0;
    long currTime = 0;
    int dt = 0;
    cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
    while (key != 'q') {
       if (zed.grab(runtime_parameters) == SUCCESS) {
           TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);
            if (tracking_state == TRACKING_STATE_OK)
            {
              if (index >=20)
              {
                transformPose(camera_pose.pose_data, translation_left_to_center); // Get the pose at the center of the camera (baseline/2 on X axis)

                // Get quaternion, rotation and translation
                sl::float4 quaternion = camera_pose.getOrientation();
                sl::float3 rotation = camera_pose.getEulerAngles(); // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
                sl::float3 translation = camera_pose.getTranslation();

                myfile << dt << ", " << translation[2] << ", " << translation[0] << ", " << translation[1] << ", " << rotation[2] << ", " << rotation[0] << ", " << rotation[1] << endl;
                //cout << translation[2] << ", " << translation[0] << ", " << translation[1] << endl;
                //cout << rotation[2] << ", " << rotation[0] << ", " << rotation[1] << endl;

                // Retrieve the left image, depth image in half-resolution
                zed.retrieveImage(image_zed_left, VIEW_LEFT, MEM_CPU, new_width, new_height);
                zed.retrieveImage(image_zed_right, VIEW_RIGHT, MEM_CPU, new_width, new_height);
                zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);
                currTime = camera_pose.timestamp;
                dt = (currTime - prevTime);


                cout << 1000000000.0/dt<< endl;
                prevTime = currTime;

                // Display image and depth using cv:Mat which share sl:Mat data
                // cv::imshow("Image_l", image_ocv_left);
                // cv::imshow("Image_r", image_ocv_right);
                // cv::imshow("Depth", depth_image_ocv);
                //cout << to_string(index) + "left.jpg" << endl;

                cv::imwrite("../Data/left/" + to_string(index-20) + ".jpg", image_ocv_left);
                cv::imwrite("../Data/right/" + to_string(index-20) + ".jpg", image_ocv_right);
                // Handle key event
                key = cv::waitKey(10);
              }

              index++;
              //processKeyEvent(zed, key);
            }

        }
    }


    myfile.close();
    // Close the camera
    zed.close();
    return 0;
}

void transformPose(sl::Transform &pose, float tx) {
    sl::Transform transform_;
    transform_.setIdentity();
    // Move the tracking frame by tx along the X axis
    transform_.tx = tx;
    // Apply the transformation
    pose = Transform::inverse(transform_) * pose * transform_;
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
