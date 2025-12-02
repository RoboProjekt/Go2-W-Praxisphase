#include <memory>
#include <chrono>
#include <iostream>
#include <unistd.h> // Für sleep()

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"

// Unitree SDK Includes
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using std::placeholders::_1;

class Go2DriverNode : public rclcpp::Node
{
public:
    Go2DriverNode() : Node("go2_driver_node")
    {
        // 1. Subscriber mit kompatiblem QoS (Reliable, damit Teleop und Treiber sich verstehen)
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Go2DriverNode::topic_callback, this, _1));

        // 2. SDK Init
        sport_client_.SetTimeout(10.0f);
        sport_client_.Init();

        // 3. START-SEQUENZ (Der wichtige Teil!)
        RCLCPP_INFO(this->get_logger(), "--------------------------------");
        RCLCPP_INFO(this->get_logger(), "Initialisiere Roboter-Status...");

        // A. Aufstehen (falls er liegt)
        RCLCPP_INFO(this->get_logger(), "-> Sende RecoveryStand (Aufstehen)...");
        int res_recovery = sport_client_.RecoveryStand();
        if(res_recovery != 0) {
            RCLCPP_WARN(this->get_logger(), "Warnung: RecoveryStand Code %d", res_recovery);
        }
        
        // Warten, bis er steht (wichtig!)
        sleep(2); 

        // B. Balance Modus erzwingen
        RCLCPP_INFO(this->get_logger(), "-> Sende BalanceStand (Fahrbereit machen)...");
        int res_balance = sport_client_.BalanceStand();
        if(res_balance == 0) {
            RCLCPP_INFO(this->get_logger(), "-> Erfolg: Roboter ist im Balance-Modus.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "-> Fehler: BalanceStand Code %d", res_balance);
        }

        RCLCPP_INFO(this->get_logger(), "--------------------------------");
        RCLCPP_INFO(this->get_logger(), "Treiber bereit. Warte auf cmd_vel Befehle...");
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = msg->linear.x;
        float vy = msg->linear.y;
        float vyaw = msg->angular.z;

        // Befehl senden
        int res = sport_client_.Move(vx, vy, vyaw);

        // Logging für Diagnose (damit du siehst, ob er Code 0 sendet)
        // Tipp: Um das Log nicht zu fluten, kannst du dies später auskommentieren
        RCLCPP_INFO(this->get_logger(), "Move(%.2f, %.2f, %.2f) -> Res: %d", vx, vy, vyaw, res);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    unitree::robot::go2::SportClient sport_client_;
};

int main(int argc, char * argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2DriverNode>());
    rclcpp::shutdown();
    return 0;
}
