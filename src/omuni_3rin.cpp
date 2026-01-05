#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/ems.hpp"

/*

     +
     ^
     |
+ <- o -> -
     |  
     v
     -

y軸はいつも通り、x軸は左右反転
*/

// ボタン
#define A 0
#define B 1
#define X 2
#define Y 3
#define LB 4
#define RB 5
#define BACK 6
#define START 7
#define home_btn 8
#define L_stick_btn 9
#define R_stick_btn 10

// 軸
#define L_stick_x 0
#define L_stick_y 1
#define R_stick_x 2
#define R_stick_y 3
#define LT 4   // 押してない状態で 1 -> 最後まで押して -1
#define RT 5   // 押してない状態で 1 -> 最後まで押して -1
#define D_pad_x 6
#define D_pad_y 7

// 制御モード番号
#define CURRENT 0
#define VELOCITY 1
#define POSITION 2 


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
        handle_ems(msg);
        publish_motor_cmd(msg);
    }

    void handle_ems(const sensor_msgs::msg::Joy::SharedPtr msg){
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
    }

    void publish_motor_cmd(const sensor_msgs::msg::Joy::SharedPtr msg){
        robomas_package_2::msg::MotorCmdArray can0;
        robomas_package_2::msg::MotorCmdArray can1;
        robomas_package_2::msg::MotorCmd can0_motor;
        robomas_package_2::msg::MotorCmd can1_motor;

        can0_motor.id = 1;
        can0_motor.mode = VELOCITY;
        can0_motor.value = msg->axes[L_stick_y] * 1000.0;
        can0.cmds.push_back(can0_motor);
        
        can0_motor.id = 2;
        can0_motor.mode = VELOCITY;
        can0_motor.value = msg->axes[L_stick_x] * -1000.0;
        can0.cmds.push_back(can0_motor);

        can0_motor.id = 3;
        can0_motor.mode = POSITION;
        if(msg->buttons[A]) can0_motor.value = 100.0;
        else if(msg->buttons[B]) can0_motor.value = -100.0;
        else can0_motor.value = 0.0;
        can0.cmds.push_back(can0_motor);

        
        can1_motor.id = 1;
        can1_motor.mode = VELOCITY;
        can1_motor.value = msg->axes[R_stick_y] * 1000.0;
        can1.cmds.push_back(can1_motor);

        can1_motor.id = 2;
        can1_motor.mode = VELOCITY;
        can1_motor.value = msg->axes[R_stick_x] * -1000.0;
        can1.cmds.push_back(can1_motor);

        can1_motor.id = 3;
        can1_motor.mode = POSITION;
        if(msg->buttons[A]) can1_motor.value = 100.0;
        else if(msg->buttons[B]) can1_motor.value = -100.0;
        else can1_motor.value = 0.0;
        can0.cmds.push_back(can1_motor);

        pub_motor_1_->publish(can0);
        pub_motor_2_->publish(can1);
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