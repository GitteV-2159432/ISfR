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

#include "tcp_socket/Tcp_Client.hpp"
#include "tcp_socket/Buffer.hpp"
#include "tcp_socket/data_structure.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rikkert_msgs/msg/platform_data.hpp"
#include "rikkert_msgs/msg/wheel_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

/**
 * Callback function to receive data from the TCP server
 * @param obj the class object this function is bound to
 * @param data_buffer data from the TCP connection
*/
void callbackFunc(void* obj, Buffer<uint8_t> *data_buffer);
/**
 * Callback function called when the tcp connection is made
 * @param ip connection ip address
*/
void onConnect(const char ip[]);

/**
 * Callback function called when the tcp connection is lost
 * @param ip connection ip address
*/
void onDisconnect(const char ip[]);

/**
 * TCP client class
*/
class TCPClient
{
public:
    /**
     * Class constructor
     * @param node the ros node object used for handling ros-components
    */
    TCPClient(rclcpp::Node::SharedPtr node);
    /**
     * Class destructor
    */
    ~TCPClient();

    //Class variables
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rikkert_msgs::msg::PlatformData>::SharedPtr platform_data_pub_;
    rclcpp::Publisher<rikkert_msgs::msg::WheelVelocity>::SharedPtr wheel_vel_pub_;
    rclcpp::Subscription<rikkert_msgs::msg::WheelVelocity>::SharedPtr wheel_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_twist_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr acknowledge_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoder_server_;
    rclcpp::TimerBase::SharedPtr velocity_timer_;
    geometry_msgs::msg::Twist latest_joy_twist_, latest_nav_twist_, zero_twist_;
    std::shared_ptr<std::thread> run_thread_;
    TCP_Client client_;
    std::string ip_;
    uint16_t port_;
    DataPlatform current_platform_data_; 
    DataPlatform previous_platform_data_; 
    DataROS current_ros_data_;

private:

    /**
     * Thread function sending back data to the TCP-server
    */
    void threadCallback();

    /**
     * Callback function to listen to the wheel velocity topic
     * @param msg the message on the topic
    */
    void wheelVelCallback(const rikkert_msgs::msg::WheelVelocity::SharedPtr msg);

    void velToPlatform();

    /**
     * Callback function to listen to the twist topic while teleoperating
     * @param msg the message on the topic
    */
    void joyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * Callback function to listen to the twist topic while autonomously navigating
     * @param msg the message on the topic
    */
    void navTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * Callback function for the acknowledgement service 
     * @param req the request component of the service
     * @param res the response component of the service
    */
    void ackServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr res);

    /**
     * Callback function to reset the value of the encoders of the platform
     * @param req the request component of the service
     * @param res the response component of the service
    */
    void resetEncoderCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                              std_srvs::srv::Trigger::Response::SharedPtr res);

    //variables
    double wheel_base_ = 0.63;
    double wheel_radius_ = 0.17;
};
