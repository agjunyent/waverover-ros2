#pragma once

#include <string>
#include <queue>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "error.hpp"

using namespace std::chrono_literals;

namespace waverover_helpers
{
    template <typename T>
    static T setAndGetParameter(const rclcpp::Node::SharedPtr node, const std::string& parameter_name, const T default_value)
    {
        node->declare_parameter(parameter_name, default_value);
        auto parameter_value = node->get_parameter(parameter_name);
        auto value = parameter_value.get_value<T>();
        return value;
    }

    template <typename T>
    static T setAndGetParameter(const rclcpp::Node::SharedPtr node, const std::string& param_name)
    {
        return setAndGetParameter<T>(node, param_name, T{});
    }

    template <typename T>
    static std::map<std::string, T> setAndGetParameters(const rclcpp::Node::SharedPtr node, const std::string& parameters_prefix, const std::map<std::string, T> parameters_map)
    {
        std::map<std::string, T> output_parameters = parameters_map;
        node->declare_parameters(parameters_prefix, output_parameters);
        node->get_parameters(parameters_prefix, output_parameters);
        return output_parameters;
    }

    template <typename T>
    static ErrorOr<std::shared_ptr<typename T::Response>> createAndGetServiceResponse(const rclcpp::Node::SharedPtr node, const std::string& service_name)
    {
        typename rclcpp::Client<T>::SharedPtr service_client = node->create_client<T>(service_name);

        auto request = std::make_shared<typename T::Request>();

        while (!service_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return Error("interrupted");
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << service_name << " not available, waiting again...");
        }

        auto result = service_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            return response;
        }
        else
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service " << service_name);
            return Error("failed");
        }
    }

    template <typename T>
    static ErrorOr<std::shared_ptr<typename T::Response>> createAndGetServiceResponse(const rclcpp::Node::SharedPtr node, const std::string& service_name, std::shared_ptr<typename T::Request> request)
    {
        typename rclcpp::Client<T>::SharedPtr service_client = node->create_client<T>(service_name);

        while (!service_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return Error("interrupted");
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << service_name << " not available, waiting again...");
        }

        auto result = service_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            return response;
        }
        else
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service " << service_name);
            return Error("failed");
        }
    }

    template <typename T>
    static ErrorOr<bool> createAndGetServiceResponse(const rclcpp::Node::SharedPtr node, const std::string& service_name, std::shared_ptr<typename T::Request> request, typename rclcpp::Client<T>::CallbackType callback)
    {
        typename rclcpp::Client<T>::SharedPtr service_client = node->create_client<T>(service_name);

        while (!service_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return Error("interrupted");
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << service_name << " not available, waiting again...");
        }

        auto result = service_client->async_send_request(request, callback);
        return true;
    }

    template <typename T>
    static typename rclcpp::Client<T>::SharedPtr getClientAsyncCallback(const rclcpp::Node::SharedPtr node, const std::string& service_name, std::shared_ptr<typename T::Request> request, typename rclcpp::Client<T>::CallbackType callback)
    {
        typename rclcpp::Client<T>::SharedPtr service_client = node->create_client<T>(service_name);

        while (!service_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return service_client;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << service_name << " not available, waiting again...");
        }

        service_client->async_send_request(request, callback);
        return service_client;
    }

    struct SensorData
    {
        double data_x;
        double data_y;
        double data_z;
        int accuracy = 0;

        SensorData() = default;
        SensorData(double x, double y, double z) :
            data_x(x), data_y(y), data_z(z)
        {

        }

        // SensorData& operator= (const SensorData& rhs)
        // {
        //     data_x = rhs.data_x;
        //     data_y = rhs.data_y;
        //     data_z = rhs.data_z;
        //     accuracy = rhs.accuracy;
        //     return *this;
        // }

        SensorData operator+ (const SensorData& rhs) const
        {
            return SensorData(
                data_x + rhs.data_x,
                data_y + rhs.data_y,
                data_z + rhs.data_z
            );
        }

        SensorData& operator+= (const SensorData& rhs)
        {
            data_x = data_x + rhs.data_x;
            data_y = data_y + rhs.data_y;
            data_z = data_z + rhs.data_z;
            return *this;
        }

        SensorData operator- (const SensorData& rhs) const
        {
            return SensorData(
                data_x - rhs.data_x,
                data_y - rhs.data_y,
                data_z - rhs.data_z
            );
        }

        SensorData& operator-= (const SensorData& rhs)
        {
            data_x = data_x - rhs.data_x;
            data_y = data_y - rhs.data_y;
            data_z = data_z - rhs.data_z;
            return *this;
        }

        SensorData operator/ (const size_t rhs) const
        {
            return SensorData(
                data_x / rhs,
                data_y / rhs,
                data_z / rhs
            );
        }
    };

    struct OrientationData
    {
        double data_x;
        double data_y;
        double data_z;
        double data_w;
        int accuracy = 0;

        OrientationData() = default;
        OrientationData(double x, double y, double z, double w) :
            data_x(x), data_y(y), data_z(z), data_w(w)
        {

        }

        friend OrientationData operator+ (OrientationData lhs, const OrientationData& rhs)
        {
            OrientationData data;
            data.data_x = lhs.data_x + rhs.data_x;
            data.data_y = lhs.data_y + rhs.data_y;
            data.data_z = lhs.data_z + rhs.data_z;
            data.data_w = lhs.data_w + rhs.data_w;
            return data;
        }

        OrientationData& operator+= (const OrientationData& rhs)
        {
            this->data_x += rhs.data_x;
            this->data_y += rhs.data_y;
            this->data_z += rhs.data_z;
            this->data_w += rhs.data_w;
            return *this;
        }

        friend OrientationData operator- (OrientationData lhs, const OrientationData& rhs)
        {
            OrientationData data;
            data.data_x = lhs.data_x - rhs.data_x;
            data.data_y = lhs.data_y - rhs.data_y;
            data.data_z = lhs.data_z - rhs.data_z;
            data.data_w = lhs.data_w - rhs.data_w;
            return data;
        }

        OrientationData& operator-= (const OrientationData& rhs)
        {
            this->data_x -= rhs.data_x;
            this->data_y -= rhs.data_y;
            this->data_z -= rhs.data_z;
            this->data_w -= rhs.data_w;
            return *this;
        }

        OrientationData& operator/ (const size_t rhs)
        {
            this->data_x /= rhs;
            this->data_y /= rhs;
            this->data_z /= rhs;
            this->data_w /= rhs;
            return *this;
        }
    };

    template <typename T, int MaxLen>
    class RunningAverage : public std::deque<T>
    {
    public:
        void push(const T& value)
        {
            if (is_full())
            {
                total_sum = total_sum - this->front();
                this->pop_front();
            }
            total_sum = total_sum + value;
            this->push_back(value);
        }
        void fill(const T& value)
        {
            this->clear();
            for (int i = 0; i < MaxLen; ++i)
            {
                this->push_back(value);
            }
            total_sum = value;
        }
        T mean()
        {
            return total_sum / MaxLen;
        }
        bool is_full() const
        {
            return this->size() == MaxLen;
        }
        bool is_empty() const
        {
            return this->size() == 0;
        }
    private:
        T total_sum{};
    };

    static inline std::string getHomeDirectory()
    {
        static const std::string homepath = getenv("HOME");
        return homepath;
    }

    static geometry_msgs::msg::Vector3 getVector3MsgFromSensorData(const SensorData sensor_data)
    {
        geometry_msgs::msg::Vector3 msg;
        msg.x = sensor_data.data_x;
        msg.y = sensor_data.data_y;
        msg.z = sensor_data.data_z;

        return msg;
    }

    static SensorData getSensorDataFromVector3Msg(const geometry_msgs::msg::Vector3::SharedPtr& msg)
    {
        return SensorData(msg->x, msg->y, msg->z);
    }

    static geometry_msgs::msg::Quaternion getQuaternionMsgFromOrientationData(const OrientationData orientation_data)
    {
        geometry_msgs::msg::Quaternion msg;
        msg.x = orientation_data.data_x;
        msg.y = orientation_data.data_y;
        msg.z = orientation_data.data_z;
        msg.w = orientation_data.data_w;

        return msg;
    }

    static OrientationData getOrientationDataFromQuaternionMsg(const geometry_msgs::msg::Quaternion::SharedPtr& msg)
    {
        OrientationData orientation_data(msg->x, msg->y, msg->z, msg->w);

        return orientation_data;
    }


    class CallbackWrapper
    {
    public:
        // alias
        using func_t = std::function<void(const std::string&)>;

        CallbackWrapper(const func_t& func, const std::string& id)
        : func(func), id(id)
        {
        }

        // copy-constructable
        CallbackWrapper(const CallbackWrapper&) = default;

        // equality-comparable
        bool operator==(const CallbackWrapper& other) const
        {
            return other.id == id;
        }

        // callable
        void operator()(const std::string& channel) const
        {
            if (func != nullptr)
            {
                func(channel);
            }
        }

    private:
        func_t func;
        std::string id;   // id for equality-comparing
    };
}
