/*
 * Copyright (c) 2023
 * Authors: David De Schepper and Ivo Dekker
 * Email: david.deschepper@kuleuven.be 
 * Email: ivo.dekker@kuleuven.be
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

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rikkert_msgs/msg/platform_data.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * Odometry calculator class
*/
class OdometryCalculator
{
public:
    /**
     * Constructor of class
     * @param node the ROS node for all ROS-components
    */
    OdometryCalculator(rclcpp::Node::SharedPtr node);
private:
    /**
     * Callback function to get data from the platform data topic
     * @param msg the message on the topic
    */
    void platformDataCallback(const rikkert_msgs::msg::PlatformData::SharedPtr msg);

    /**
     * Function to translate a yaw to a quaternion
     * @param yaw the yaw angle
    */
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

    //Class variables
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rikkert_msgs::msg::PlatformData>::SharedPtr platform_data_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    int32_t cur_ticks_l_ = 0;
    int32_t cur_ticks_r_ = 0;
    int32_t prev_ticks_l_ = 0;
    int32_t prev_ticks_r_ = 0;

    //Parameters
    int pulses_per_rotation_ = 1024;
    int reduction_ = 144;
    int ppr_ = pulses_per_rotation_ * reduction_;
    double angle_per_pulse_ = 2.0*M_PI/(double)ppr_;
    double wheel_radius_ = 0.17;
    double wheel_base_ = 0.63; 
    double theta_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;

    double theta_left_ = 0.0;
    double theta_right_ = 0.0;
    
};