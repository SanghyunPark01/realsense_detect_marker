#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <exception>
#include <string.h>
#include <cmath>

static cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);
double rad_to_deg(double r);
int main(int argc, char* argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::align align_to(RS2_STREAM_COLOR);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    using namespace std;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);
    cameraMatrix = (Mat1d(3, 3) << 618.53013, 0., 324.7346, 0., 616.45867, 246.24308, 0., 0., 1.);
    distCoeffs = (Mat1d(1, 5) << 0.123868, -0.281684, -0.002987, 0.000575, 0.);
    float u0 = 324.7346;
    float v0 = 246.24308;

    while (waitKey(1)<0)
    {
        //Get Frame

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        //rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frameset aligned_set = align_to.process(data);
        rs2::frame depth = aligned_set.get_depth_frame();
        auto image_rgb = frame_to_mat(aligned_set.get_color_frame());
        color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        rs2::frame bw_depth = depth.apply_filter(color_map);
        auto image_depth = frame_to_mat(bw_depth);
        //imshow("rgb", color_mat);
        //imshow("depth", near);

        Mat image_output;
        //image_rgb.copyTo(image_output);
        
        //Calibration
        undistort(image_rgb, image_output, cameraMatrix, distCoeffs);
        //imshow("calib", image_output);
        //imshow("uncalib", image_rgb);
         
        //Align Check

        imshow("depth image", image_depth);
        //imshow("rgb image", image_output);

        Mat depth_value = Mat::zeros(480, 640, CV_64F);
        
        depth_value = depth_frame_to_meters(pipe, depth);
        //auto distance_d = depth_value.at<double>(320, 240);//center 320, 240
        //cout << distance_d << endl;
        
        //Get Marker

        vector<int>ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(image_output, dictionary, corners, ids);
        //cout << corners.size() << endl;
        vector<Point2f>centers;
        vector<double>distance_c;
        //if marker exist
        if (ids.size() > 0) {
            
            //cout<<corners[0][0]<<endl;
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            for (int i = 0; i < ids.size(); i++) {
                drawFrameAxes(image_output, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
                //red : X , green : Y , blue : Z
                //
                //Mat R;
                //Rodrigues(rvecs[i], R);
                //cout << R << endl;
                //cout << "rvec : " << rvecs[i] << endl;
                //cout << "tvec : " << tvecs[i] << endl;//x,y,z camera_frame to marker frame
            }

            for (const auto& corner : corners)
            {
                Point2f center(0.f, 0.f);
                // corners.size() == 4
                for (const auto& corner_a : corner) {
                    center += corner_a;
                }
                center /= 4.f;
                int center_x = (int)center.x;
                int center_y = (int)center.y;
                //cout << center.x << ", " << center.y << endl;

                //Get Marker Depth
                if (center_x > 0 && center_y > 0) {
                    auto d = depth_value.at<double>(center_y,center_x);
                    //cout << to_string(d) << endl;
                    distance_c.push_back(d);
                }
                centers.push_back(center);

            }
            //visualize
            for (int i = 0; i < centers.size(); i++) {
                circle(image_output, centers[i], 20, Scalar(255, 255, 255)); 
                putText(image_output, to_string(distance_c[i])+"m", Point(25, 25), 1, 1, Scalar(255, 255, 100));
            }
            aruco::drawDetectedMarkers(image_output, corners, ids);
        }else{ 
            putText(image_output, "No Marker !!", Point(25, 60), 1, 5, Scalar(0, 0, 255));
        }
        imshow("result", image_output);

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}
double rad_to_deg(double r)
{
    double d;
    
    d = r * 180 / 3.14;
    if (d < -180) {
        d += 360;
    }else if(d > 180){
        d -= 360;
    }

    return d;
}