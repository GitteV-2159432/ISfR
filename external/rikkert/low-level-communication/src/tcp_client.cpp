/*
 * Copyright (c) 2023
 * Authors: Ivo Dekker and David De Schepper
 * Email: ivo.dekker@kuleuven.be
 * Email: david.deschepper@kuleuven.be 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "revpi_communication/tcp_client.hpp"

TCPClient *client;

TCPClient::TCPClient(rclcpp::Node::SharedPtr node)
{
    // Init data container
    current_platform_data_.state = systemState::ERROR;
    current_platform_data_.reset_done = 1;
    current_ros_data_.ACK = 0;
    current_ros_data_.encoder_reset = 0;
    zero_twist_.linear.x = 0.0;
    zero_twist_.linear.y = 0.0;
    zero_twist_.linear.z = 0.0;
    zero_twist_.angular.x = 0.0;
    zero_twist_.angular.y = 0.0;
    zero_twist_.angular.z = 0.0;
    latest_joy_twist_ = zero_twist_;
    latest_nav_twist_ = zero_twist_;


    node_ = node;
    platform_data_pub_ = node_->create_publisher<rikkert_msgs::msg::PlatformData>("platform_data", 1);
    wheel_vel_pub_ = node_->create_publisher<rikkert_msgs::msg::WheelVelocity>("wheel_velocity", 1);
    // wheel_vel_sub_ = node_->create_subscription<rikkert_msgs::msg::WheelVelocity>("wheel_velocity", 1, std::bind(&TCPClient::wheelVelCallback, this, std::placeholders::_1));
    // To be used with navigation stack
    nav_twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 1, std::bind(&TCPClient::navTwistCallback, this, std::placeholders::_1));
    // To be used with joystick
    joy_twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("joy_twist", 1, std::bind(&TCPClient::joyTwistCallback, this, std::placeholders::_1));
    acknowledge_server_ = node_->create_service<std_srvs::srv::Trigger>("acknowledgement", std::bind(&TCPClient::ackServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    reset_encoder_server_ = node_->create_service<std_srvs::srv::Trigger>("reset_encoders", std::bind(&TCPClient::resetEncoderCallback, this, std::placeholders::_1, std::placeholders::_2));
    velocity_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TCPClient::velToPlatform, this));
    run_thread_ = std::make_shared<std::thread>(std::bind(&TCPClient::threadCallback, this));
    ip_ = "192.168.69.10";
    port_ = 5050;
    client_.set_callback(this, callbackFunc);
    client_.set_on_connect(onConnect);
    client_.set_on_disconnect(onDisconnect);
    client_.start(ip_, port_);
}
TCPClient::~TCPClient()
{
    client_.stop();
    run_thread_->detach();
}

void TCPClient::threadCallback()
{
    while (rclcpp::ok())
    {
        uint8_t *data = new uint8_t[sizeof(DataROS)];
        DataROS temp_ros_data = current_ros_data_;

        switch(current_platform_data_.state)
        {
            case systemState::EMS:
                temp_ros_data.v_l = 0.0;
                temp_ros_data.v_r = 0.0;
                temp_ros_data.ACK = 0;
                break;
            case systemState::ACK:
                temp_ros_data.v_l = 0.0;
                temp_ros_data.v_r = 0.0;
                break;
            case systemState::RUNNING:
                if(previous_platform_data_.state == systemState::ACK)
                {
                    current_ros_data_.ACK = 0;
                }
                break;

            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown system state received");
        }
        memcpy(data, &temp_ros_data, sizeof(DataROS));
        client_.write(data, sizeof(DataROS));
        delete data;
        previous_platform_data_ = current_platform_data_;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// void TCPClient::threadCallback()
// {
//     while (rclcpp::ok())
//     {
//         uint8_t *data = new uint8_t[sizeof(DataROS)];
//         if (current_platform_data_.state == systemState::ACK)
//         {
//             while (current_ros_data_.ACK == 0 && current_platform_data_.state == systemState::ACK)
//             {
//             }
//             if (current_platform_data_.state == systemState::ACK)
//             {
//                 memcpy(data, &current_ros_data_, sizeof(DataROS));
//                 client_.write(data, sizeof(DataROS));

//                 // Clear the acknowledgement
//                 current_ros_data_.ACK = 0;
//                 while (current_platform_data_.state == systemState::ACK)
//                 {
//                 }
//             }
//         }
//         else if (current_platform_data_.state == systemState::RUNNING)
//         {
//             memcpy(data, &current_ros_data_, sizeof(DataROS));
//             client_.write(data, sizeof(DataROS));
//         }
//         // else if (current_platform_data_.state == systemState::EMS)
//         // {
//         //     //Allow parsing of the encoder reset
//         //     DataROS temp_ros_data = current_ros_data_;
//         //     temp_ros_data.v_l = 0.0;
//         //     temp_ros_data.v_r = 0.0;
//         //     memcpy(data, &temp_ros_data, sizeof(DataROS));
//         //     client_.write(data, sizeof(DataROS));
//         // }
//         delete data;
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
// }

// void TCPClient::wheelVelCallback(const rikkert_msgs::msg::WheelVelocity::SharedPtr msg)
// {
//     if(current_platform_data_.state == systemState::RUNNING)
//     {
//         current_ros_data_.v_l = msg->w_l;
//         current_ros_data_.v_r = msg->w_r;
//     }
//     else
//     {
//         current_ros_data_.v_l = 0.0;
//         current_ros_data_.v_r = 0.0;
//     }
// }

void TCPClient::velToPlatform()
{
    geometry_msgs::msg::Twist output_twist;
    if(latest_joy_twist_ != zero_twist_)
    {
        output_twist = latest_joy_twist_;
    }
    else
    {
        output_twist = latest_nav_twist_;
    }

    rikkert_msgs::msg::WheelVelocity wheelVel;
    //Translate to robot left & right wheel velocity
    wheelVel.w_l = (output_twist.linear.x - output_twist.angular.z * wheel_base_/2)/wheel_radius_; //Wheel velocity in rad/s
    wheelVel.w_r = (output_twist.linear.x + output_twist.angular.z * wheel_base_/2)/wheel_radius_;
    wheel_vel_pub_->publish(wheelVel);

    if(current_platform_data_.state == systemState::RUNNING)
    {
        current_ros_data_.v_l = wheelVel.w_l;
        current_ros_data_.v_r = wheelVel.w_r;
    }
    else
    {
        current_ros_data_.v_l = 0.0;
        current_ros_data_.v_r = 0.0;
    }
}

void TCPClient::joyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_joy_twist_ = *msg;
}

void TCPClient::navTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_nav_twist_ = *msg;
}

void TCPClient::ackServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr res)
{
    // Set acknowledgement to 1, ready for sending to platform
    if (current_platform_data_.state == systemState::ACK)
    {
        current_ros_data_.ACK = 1;
    }
    res->success = true;
}

void TCPClient::resetEncoderCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                     std_srvs::srv::Trigger::Response::SharedPtr res)
{
    current_ros_data_.encoder_reset = 1;
    res->success = true;
}

void callbackFunc(void *obj, Buffer<uint8_t> *data_buffer)
{
    client = (TCPClient *)obj;
    // std::cout << "Received: " << data_buffer->size() << std::endl;
    DataPlatform data_rec;
    while (data_buffer->size() >= sizeof(DataPlatform))
    {
        data_buffer->pull(&data_rec, sizeof(DataPlatform));
    }
    if(data_rec.reset_done == 1)
    {
        client->current_ros_data_.encoder_reset = 0;
    }
    client->current_platform_data_ = data_rec;
    rikkert_msgs::msg::PlatformData data;
    data.header.stamp = client->node_->now();
    //TODO: add TF frames
    data.ticks_l = data_rec.ticks_l;
    data.ticks_r = data_rec.ticks_r;
    data.voltage = data_rec.voltage;
    data.temp = data_rec.temp;
    data.state = (int)data_rec.state;

    client->platform_data_pub_->publish(data);
    data_buffer->clear();
}

void onConnect(const char ip[])
{
    std::cout << "Connected to IP: " << ip << std::endl;
}
void onDisconnect(const char ip[])
{
    std::cout << "Disconnected to IP: " << ip << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("revpi_client_node");
    TCPClient client(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}