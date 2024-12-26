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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rikkert_msgs/msg/wheel_velocity.hpp"

class VelocityCalculator
{
public:
    VelocityCalculator(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("twist", 1);
        // wheel_vel_pub_ = node_->create_publisher<rikkert_msgs::msg::WheelVelocity>("wheel_velocity", 1);
        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&VelocityCalculator::joyCallback, this, std::placeholders::_1));
    }
private:

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // rikkert_msgs::msg::WheelVelocity wheelvel;
        geometry_msgs::msg::Twist output;
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        output.linear.z = 0.0;
        output.angular.x = 0.0;
        output.angular.y = 0.0;
        output.angular.z = 0.0;
        if(!msg->buttons[5])
        {
            twist_pub_->publish(output);
            // wheelvel.w_l = 0.0; //Wheel velocity in rad/s
            // wheelvel.w_r = 0.0;

            // wheel_vel_pub_->publish(wheelvel);
            return;
        }

        //Read in triggers for forward/backward motion
        double forwards = (-(msg->axes[5] - 1))/2;
        double backwards = (-(msg->axes[2] - 1))/2;
        double steering = msg->axes[0];

        double lin  = forwards - backwards;
        double rot = msg->axes[0];

        if(lin > trigger_deadzone_ || lin < -trigger_deadzone_)
        {
            output.linear.x = (lin/std::fabs(lin)) * max_vel_lin_ * (std::fabs(lin)/(std::fabs(lin) + std::fabs(rot))) * std::fabs(lin); 
        }
        if(rot > stick_deadzone_ || rot < -stick_deadzone_)
        {
            output.angular.z = (rot/std::fabs(rot))* max_vel_ang_* (std::fabs(rot)/(std::fabs(lin) + std::fabs(rot))) * std::fabs(rot);
        }
        //Read left stick for angular velocity

        twist_pub_->publish(output);

        //Translate to robot left & right wheel velocity
        // wheelvel.w_l = (output.linear.x - output.angular.z * wheel_base_/2)/wheel_radius_; //Wheel velocity in rad/s
        // wheelvel.w_r = (output.linear.x + output.angular.z * wheel_base_/2)/wheel_radius_;

        // wheel_vel_pub_->publish(wheelvel);
    }

    //Class variables
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<rikkert_msgs::msg::WheelVelocity>::SharedPtr wheel_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    //Velocity parameters
    double max_vel_lin_ = 0.425;
    double max_vel_ang_ = 1.3492/2;
    double trigger_deadzone_ = 0.05;
    double stick_deadzone_ = 0.05;
    // double wheel_base_ = 0.63;
    // double wheel_radius_ = 0.17;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("velocity_calculator_node");
    VelocityCalculator vc(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}