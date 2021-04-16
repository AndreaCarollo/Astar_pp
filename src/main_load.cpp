#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"
#include "./lib/configurator.h"
#include "./lib/rs_stream.h"
#include "./lib/PathPlanning.h"

using namespace std;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~ main ~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


int main(int argc, char **argv)
{
    cv::Point3f target_point3D = cv::Point3f(0.0, 0.0, 1.0);
    // ~~~~~~~~~~~~~~~~~~~ CONFIGURATION FILE ~~~~~~~~~~~~~~~~~~~ //

    // Create the configurator object and parse conf.ini file
    ConfigReader *p = ConfigReader::getInstance();
    p->parseFile("../config.ini");

    int IR_WIDTH = 640;
    int IR_HEIGHT = 480;
    int RGB_WIDTH = 640;
    int RGB_HEIGHT = 480;
    int FRAME_RATE = 30;

    // ~~~~~~~~~~~~~~~~~~~~ INITIALIZATIONS ~~~~~~~~~~~~~~~~~~~~~ //

    // Others - OPENCV
    int x_cv, y_cv;
    float x_rel, y_rel;
    cv::Point target_point;
    // cv::Point3f target_point3D;

    // Others - visualization and timing
    PntCld::Ptr cloud_tmp;
    std::vector<float> durations_pathpl, durations_rgb_acq, durations_visu;
    PntCldV::Ptr viewer(new PntCldV("3D Viewer"));
    viewer->initCameraParameters();

    // ~~~~~~~~~~~~~~~~~~~~ START THE STREAM ~~~~~~~~~~~~~~~~~~~~ //

    // Add desired streams to configuration
    // camSettings(&cfg, p);                                         // Enable stream from the camera
    // cfg.enable_device_from_file("../d435i_walk_around.bag");      // Enable stream from a recordered device (.bag file)
    //
    // The code works using the d435i_walk_around.bag file. This file is an example file found in the web site of realsense
    //
    //                  https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md
    //
    //               --> Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU) <--
    //
    // NOTE : The configuration file is tuned over this stream file.

    // Initialize the Path Planning stream
    PathPlanning plan = PathPlanning(p);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../test_pcd_4.pcd", *cloud_in);

    // LOOP
    int contatore = 0;
    while (!viewer->wasStopped()) // The visualizer stops, stop the code
    {

        // // ~~~~~~~~~~~~~~~ Path Planning Part ~~~~~~~~~~~~~~~~~~~~ //

        auto start_pathpl = std::chrono::high_resolution_clock::now();

        // Update the Path Planning class -> all others operations are done inside the update function
        plan.update(&target_point3D, cloud_in);

        auto stop_pathpl = std::chrono::high_resolution_clock::now();
        auto duration_pathpl = std::chrono::duration_cast<std::chrono::microseconds>(stop_pathpl - start_pathpl);

        durations_pathpl.push_back((float)duration_pathpl.count() / 1000);

        // // ~~~~~~~~~~~~~~~ Visualization Part ~~~~~~~~~~~~~~~~~~~~ //

        auto start_visu = std::chrono::high_resolution_clock::now();

        // Add a cube to the visualizer that works as a marker
        viewer->addCube(plan.refPnt.x - 0.030, plan.refPnt.x + 0.030,
                        plan.refPnt.y - 0.030, plan.refPnt.y + 0.030,
                        plan.refPnt.z - 0.030, plan.refPnt.z + 0.030,
                        1.0, 0.0, 0.0);

        PCViewer(cloud_in, viewer);
        viewer->addPointCloud(plan.plane->plane_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_in, 0, 255, 0), "Plane");
        viewer->addCoordinateSystem(1, "RF_cam");
        viewer->addCoordinateSystem(1, plan.plane->transf_mtx.inverse(), "RF_plane");

        cv::imshow("Interface", plan.interface->interface);
        viewer->spinOnce(100); // wait for some microseconds, makes the viewer interactive

        // Remove the point cloud from the viewer
        viewer->removePointCloud("sample cloud");
        viewer->removePointCloud("Plane"); // this gives segmentation when no plane is found! anyway is only for debug
        viewer->removeShape("cube");
        viewer->removeAllCoordinateSystems();

        auto stop_visu = std::chrono::high_resolution_clock::now();
        auto duration_visu = std::chrono::duration_cast<std::chrono::microseconds>(stop_visu - start_visu);

        durations_visu.push_back((float)duration_visu.count() / 1000);
        cv::waitKey(10);
    }

    float t_rgb_acq = accumulate(durations_rgb_acq.begin(), durations_rgb_acq.end(), 0.0) / durations_rgb_acq.size();
    float t_pathpl = accumulate(durations_pathpl.begin(), durations_pathpl.end(), 0.0) / durations_pathpl.size();
    float t_visu = accumulate(durations_visu.begin(), durations_visu.end(), 0.0) / durations_visu.size();

    cout << "Timing :" << endl;
    cout << "   PathPlanning algorithm time :  " << t_pathpl << "\t[ms]" << endl;
    cout << endl;
    cout << "   Visualization time          :  " << t_visu << "\t[ms]" << endl;
    cout << endl
         << endl;
    cout << "NOTE : The most of visualization time is caused by the point cloud (NOT REALLY NEEDED)" << endl;

    return (0);
}
