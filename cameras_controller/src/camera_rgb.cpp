#include "cameras_controller/camera_rgb.hpp"

//==============================================================================
//				  Constructors
//==============================================================================

CameraRGB::CameraRGB(const rclcpp::Node::SharedPtr& node, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution)
    : Camera(node, port_or_ip, fps, resolution)
{
    gstreamer_input = "v4l2src device=" + port_or_ip + " io-mode=2 ! image/jpeg, width=(int)" + std::to_string(resolution[0]) + ", height=(int)" + std::to_string(resolution[1]) + ", framerate=" + std::to_string(fps) + "/1 ! nvv4l2decoder mjpeg=1 enable-max-performance=1 ! queue ! nvvidconv ! video/x-raw, format=BGRx ! queue ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=1";

    frame.create(resolution[1], resolution[0], CV_8UC3);
    timestamp = node->now();

    img_msg.width = frame.cols;
    img_msg.height = frame.rows;
    img_msg.header.stamp = timestamp;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = 0;
    img_msg.step = frame.step[0];

    memcpy_size = frame.step * frame.rows;
    img_msg.data.resize(memcpy_size);

    int port_number = getPortNumber();
    if (port_number == -1)
    {
        return;
    }
    cap = cv::VideoCapture(port_number);
    cap.release();
    cap = cv::VideoCapture(gstreamer_input, cv::CAP_GSTREAMER);
    cap.read(frame);

    is_reading = true;
    thread_read_frames = std::thread(std::bind(&CameraRGB::readFramesThread, this));
}

//==============================================================================
//				  Destructors
//==============================================================================

CameraRGB::~CameraRGB()
{
    cap.release();
}

//==============================================================================
//			   Public methods
//==============================================================================
void CameraRGB::shutdownCamera()
{
    is_reading = false;
    thread_read_frames.join();
}

void CameraRGB::restartCamera()
{
    shutdownCamera();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    int port_number = getPortNumber();
    if (port_number == -1)
    {
        return;
    }
    cap.release();
    cap = cv::VideoCapture(port_number);
    cap.release();
    cap = cv::VideoCapture(gstreamer_input, cv::CAP_GSTREAMER);
    cap.read(frame);

    is_reading = true;
    thread_read_frames = std::thread(std::bind(&CameraRGB::readFramesThread, this));
}

//==============================================================================
//			   Private methods
//==============================================================================

void CameraRGB::readFramesThread()
{
    while (is_reading)
    {
        bool ret = cap.read(frame);
        if (!ret)
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "Camera RGB failed!");
            cap.release();
            RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for watchdog to restart");
            is_reading = false;
            break;
        }
        readNewFrame();
        timestamp = node->now();
        frame_ready = true;
    }
}

sensor_msgs::msg::Image::SharedPtr CameraRGB::getFrame()
{
    return cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
}

bool CameraRGB::frameReady()
{
    const bool is_ready = frame_ready;
    if (is_ready)
    {
        frame_ready = false;
    }
    return is_ready;
}

int CameraRGB::getPortNumber() const
{
    int port_number = 0;
    std::filesystem::path port_path(port_or_ip);
    if (std::filesystem::exists(port_path))
    {
        if (std::filesystem::is_symlink(port_path))
        {
            const std::string port_symlink = std::filesystem::read_symlink(port_path);
            RCLCPP_INFO_STREAM(node->get_logger(), "Found RGB camera in port " << port_symlink);
            port_number = std::atoi(&port_symlink.back());
        }
        else
        {
            port_number = std::atoi(&port_or_ip.back());
        }
    }
    else
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Port " << port_or_ip << " does not exist!!");
        port_number = -1;
    }
    return port_number;
}