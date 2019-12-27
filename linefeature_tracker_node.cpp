#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "linefeature_tracker.h"

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;

LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;


/**
 * @brief   ROS的回调函数，对新来的图像进行特征追踪，发布
 * @Description readImage()函数对新来的图像提取/匹配/发布线特征
 * @param[in]   img_msg 输入的图像
 * @return      void
*/
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    // 判断是否是第一帧
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
    }

    // frequency control, 如果图像频率低于一个值
    // 发布频率控制
    // 并不是每读入一帧图像，就要发布特征点
    // 判断间隔时间内的发布次数
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;



    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);//图像的指针
    cv::Mat show_img = ptr->image;


//     cv::imshow("lineimg",show_img); //TODO:
//     cv::waitKey(1);

    // readImage
    TicToc t_r;
    frame_cnt++;
    trackerData.readImage(ptr->image.rowRange(0 , ROW));   //TODO: rowRange(i,j) 取图像的i～j行


    // 发布线特征
    if (PUB_THIS_FRAME)
    {
        pub_count++;

        /* sensor_msgs::PointCloudPtr的格式
        std_msgs/Header header
        geometry_msgs/Point32[] points
        sensor_msgs/ChannelFloat32[] channels   */ 

        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v

        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                // 得到归一化平面上的坐标
                auto un_lines = trackerData.undistortedLineEndPoints();

                //auto &cur_lines = trackerData.curframe_->vecLine;
                auto &ids = trackerData.curframe_->lineID; //lineID

                for (unsigned int j = 0; j < ids.size(); j++)
                {

                    int p_id = ids[j]; //第j个线的id
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p; //起始点的位置
                    p.x = un_lines[j].StartPt.x;
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);
                    id_of_line.values.push_back(p_id * NUM_OF_CAM +i );
                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    //ROS_ASSERT(inBorder(cur_pts[j]));
                }
            }

        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);
        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
        pub_img.publish(feature_lines);

    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
    ROS_INFO("whole Line feature tracker processing costs: %f", mean_time);
}

int main(int argc, char **argv)
{
    //ros初始化和设置句柄
    ros::init(argc, argv, "linefeature_tracker");
    ros::NodeHandle n("~");

    //设置logger的级别。 只有级别大于或等于level的日志记录消息才会得到处理。
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //读取yaml中的一些配置参数
    readParameters(n);

    //读取每个相机实例对应的相机内参
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);

    // TODO: 开始线特征的检测
    ROS_INFO("start line feature");
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // 发布一些话题,给后端用
    pub_img = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("linefeature_img",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}
