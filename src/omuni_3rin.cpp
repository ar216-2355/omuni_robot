#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/ems.hpp"

class Omuni3Rin : public rclcpp::Node
{
public:
    Omuni3Rin() : Node("omuni_3rin"){
        sub_joy_   = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Omuni3Rin::joy_callback, this, std::placeholders::_1));
        pub_motor_1_ = this->create_publisher<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array_can0", 10);
        pub_ems_1_   = this->create_publisher<robomas_package_2::msg::Ems>("ems_tx_can0", 10);
        pub_motor_2_ = this->create_publisher<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array_can1", 10);
        pub_ems_2_   = this->create_publisher<robomas_package_2::msg::Ems>("ems_tx_can1", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if(msg->buttons[6]){
            // 非常停止モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = true;
            pub_ems_1_->publish(EMS);
            pub_ems_2_->publish(EMS);
        }
        else if(msg->buttons[7]){
            // 駆動モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = false;
            pub_ems_1_->publish(EMS);
            pub_ems_2_->publish(EMS);
        }
        
        robomas_package_2::msg::MotorCmdArray out1;
        robomas_package_2::msg::MotorCmd cmd1;

        // --- motor1: 右スティックX（axes[3]） ---
        cmd.id = 1;
        cmd.mode = 1;  // SPEED mode
        cmd.value = msg->axes[3] * 1000.0f;
        out.cmds.push_back(cmd);

        // --- motor2: 左スティックX（axes[0]） ---
        cmd.id = 2;
        cmd.mode = 1;
        cmd.value = msg->axes[0] * 1000.0f;
        out.cmds.push_back(cmd);

        // --- motor3: LT (axes[2]) ---
        // ※コントローラによって値が 1→-1 or 0→1 など違うため必要に応じて調整
        cmd.id = 3;
        cmd.mode = 1;
        cmd.value = msg->axes[2] * 1000.0f;
        out.cmds.push_back(cmd);

        pub_motor_->publish(out);

        

        RCLCPP_INFO(
            this->get_logger(),
            "m1=%.1f  m2=%.1f  m3=%.1f",
            out.cmds[0].value,
            out.cmds[1].value,
            out.cmds[2].value
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<robomas_package_2::msg::MotorCmdArray>::SharedPtr pub_motor_1_;
    rclcpp::Publisher<robomas_package_2::msg::Ems>::SharedPtr pub_ems_1_;
    rclcpp::Publisher<robomas_package_2::msg::MotorCmdArray>::SharedPtr pub_motor_2_;
    rclcpp::Publisher<robomas_package_2::msg::Ems>::SharedPtr pub_ems_2_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omuni3Rin>());
    rclcpp::shutdown();
    return 0;
}