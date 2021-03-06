// Double inclusion guard
#ifndef RS_STREAM
#define RS_STREAM

#include "./followme.h"
#include "./configurator.h"


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Stream{
    public:

        std::string stream_name;

        int w_RGB, h_RGB;
        int w_IR, h_IR;

        cv::Mat color_frame;
        cv::Mat infrared_frame;
        cv::Mat depth_frame;

        PntCld::Ptr cloud;

        pcl::PointXYZ refPnt;

        Stream(std::string stream_name, rs2::frameset *frames, ConfigReader* cfg);
        void update(rs2::frameset *frames);
        void RGB_acq();
        void IR_acq();
        void PC_acq(bool flag = false);
        void project_RGB2DEPTH(cv::Point *input);

        rs2::frame depth;
        rs2::frame color;
        rs2::frameset* frames;
    private:

        int leaf;


        cv::Mat tmp;

        rs2::frame infrared;
        rs2::stream_profile depth_profile;
        rs2::stream_profile color_profile;

        float depth_scale;

        float rgb_pixel[2];
        float depth_pixel[2];

        rs2::pointcloud pc;
        rs2::points points;

        void points_to_pcl();
};


#endif