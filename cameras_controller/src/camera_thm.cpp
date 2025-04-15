#include "cameras_controller/camera_thm.hpp"

//==============================================================================
//				  Constructors
//==============================================================================

CameraTHM::CameraTHM(const rclcpp::Node::SharedPtr& node, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution)
    : Camera(node, port_or_ip, fps, resolution)
{
    frame.create(resolution[1], resolution[0], CV_16UC1);
    timestamp = node->now();

    img_msg.width = frame.cols;
    img_msg.height = frame.rows;
    img_msg.header.stamp = timestamp;
    img_msg.encoding = "mono16";
    img_msg.is_bigendian = 0;
    img_msg.step = frame.step[0];

    memcpy_size = frame.step * frame.rows;
    img_msg.data.resize(memcpy_size);

    int port_number = getPortNumber();
    if (port_number == -1)
    {
        return;
    }

    cap = cv::VideoCapture(port_number, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    cap.read(frame);

    is_reading = true;
    thread_read_frames = std::thread(std::bind(&CameraTHM::readFramesThread, this));
}

//==============================================================================
//				  Destructors
//==============================================================================

CameraTHM::~CameraTHM()
{
    cap.release();
}

//==============================================================================
//			   Public methods
//==============================================================================
void CameraTHM::shutdownCamera()
{
    is_reading = false;
    thread_read_frames.join();
}

void CameraTHM::restartCamera()
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
    cap.read(frame);

    is_reading = true;
    thread_read_frames = std::thread(std::bind(&CameraTHM::readFramesThread, this));
}

//==============================================================================
//			   Private methods
//==============================================================================

void CameraTHM::readFramesThread()
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

sensor_msgs::msg::Image::SharedPtr CameraTHM::getFrame()
{
    return cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", frame).toImageMsg();
}

bool CameraTHM::frameReady()
{
    const bool is_ready = frame_ready;
    if (is_ready)
    {
        frame_ready = false;
    }
    return is_ready;
}

int CameraTHM::getPortNumber() const
{
    int port_number = 0;
    std::filesystem::path port_path(port_or_ip);
    if (std::filesystem::exists(port_path))
    {
        if (std::filesystem::is_symlink(port_path))
        {
            const std::string port_symlink = std::filesystem::read_symlink(port_path);
            RCLCPP_INFO_STREAM(node->get_logger(), "Found THM camera in port " << port_symlink);
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