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

#include "platform_calculations/odometry.hpp"

OdometryCalculator::OdometryCalculator(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    platform_data_sub_ = node_->create_subscription<rikkert_msgs::msg::PlatformData>("platform_data", 1, std::bind(&OdometryCalculator::platformDataCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void OdometryCalculator::platformDataCallback(const rikkert_msgs::msg::PlatformData::SharedPtr msg)
{
    // Get tick value
    cur_ticks_l_ = msg->ticks_l;
    cur_ticks_r_ = msg->ticks_r;

    // Calculate tick displacement
    int delta_l = cur_ticks_l_ - prev_ticks_l_;
    int delta_r = cur_ticks_r_ - prev_ticks_r_;
    if ((std::abs(delta_l) > 1024 * reduction_) || (std::abs(delta_r) > 1024 * reduction_))
    {
        prev_ticks_l_ = cur_ticks_l_;
        prev_ticks_r_ = cur_ticks_r_;
        return;
    }
    double omega_l = delta_l * angle_per_pulse_;
    double omega_r = delta_r * angle_per_pulse_;
    theta_left_+= omega_l; 
    theta_right_+= omega_r; 
    double v_l = omega_l * wheel_radius_;
    double v_r = omega_r * wheel_radius_;

    // Differential drive kinematics
    double v_x = (v_l + v_r) / 2.0;
    double v_y = 0.0;
    double omega_z = (-v_l / wheel_base_) + (v_r / wheel_base_);

    // Calculate pose displacement using kinematic model
    double delta_theta = omega_z;
    double delta_x = v_x * cos(theta_);
    double delta_y = v_x * sin(theta_);

    // Calculate new pose based on prev pose
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Create the odom trans message
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = node_->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = createQuaternionMsgFromYaw(theta_);

    tf_broadcaster_->sendTransform(odom_trans);

    // Create the joint state message for the two drive wheels
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = node_->now();
    
    joint_state_msg.name.resize(2);
    joint_state_msg.position.resize(2);
    joint_state_msg.velocity.resize(2);
    joint_state_msg.effort.resize(2);

    joint_state_msg.name[0] = "left_wheel_joint";
    joint_state_msg.name[1] = "right_wheel_joint";
    joint_state_msg.position[0] = theta_left_;
    joint_state_msg.position[1] = theta_right_;
    joint_state_msg.velocity[0] = v_l;
    joint_state_msg.velocity[1] = v_r;
    joint_state_msg.effort[0] = 0.0;
    joint_state_msg.effort[1] = 0.0;

    joint_state_pub_->publish(joint_state_msg);

    // Fill the odom message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = createQuaternionMsgFromYaw(theta_);
    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[8] = 0.0;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 1e3;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[8] = 0.0;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 1e3;

    // Publish odometry
    odom_pub_->publish(odom);

    // Put tick value in prev
    prev_ticks_l_ = cur_ticks_l_;
    prev_ticks_r_ = cur_ticks_r_;
}

geometry_msgs::msg::Quaternion OdometryCalculator::createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.getX();
    msg.y = q.getY();
    msg.z = q.getZ();
    msg.w = q.getW();
    return msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("odometry_calculator_node");
    OdometryCalculator oc(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}