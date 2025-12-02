/**
 * Datei: /home/bauya/ros2_ws/src/unitree_ros2/example/src/src/go2w/go2w_imu.cpp
 */

#include <memory>
#include <string>
#include <vector>
#include <cmath> // Wichtig für std::isnan

// ROS2 Header
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp" // Für Magnetometer
#include "geometry_msgs/msg/vector3_stamped.hpp"

// Unitree SDK2 Header
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/IMUState_.hpp>

// Topic Definition
#define TOPIC_LOWSTATE "rt/lowstate"

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("go2_imu_publisher")
    {
        // 1. Publisher initialisieren
        pub_imu_combined_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/accel", 10);
        pub_gyro_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/gyro", 10);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

        // 2. Unitree Subscriber für LowState initialisieren
        low_state_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
            TOPIC_LOWSTATE);

        // InitChannel bindet den Callback
        low_state_sub_->InitChannel(
            std::bind(&ImuNode::LowStateHandler, this, std::placeholders::_1), 1);

        RCLCPP_INFO(this->get_logger(), "Go2 IMU Node gestartet. Warte auf LowState...");
    }

private:
    void LowStateHandler(const void *message)
    {
        const unitree_go::msg::dds_::LowState_ *state = (const unitree_go::msg::dds_::LowState_ *)message;
        auto imu_data = state->imu_state();
        
        // --- 1. Sicherheitscheck: Quaternion ---
        // Ein Quaternion [0,0,0,0] ist mathematisch ungültig und crasht kiss_icp.
        float qw = imu_data.quaternion()[0];
        float qx = imu_data.quaternion()[1];
        float qy = imu_data.quaternion()[2];
        float qz = imu_data.quaternion()[3];

        // Prüfen, ob Quaternion leer (Summe der Quadrate ist 0) oder NaN ist
        float norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
        if (norm_sq < 0.001f || std::isnan(norm_sq)) {
            // Warnung nur einmal pro Sekunde ausgeben, um Log nicht zu fluten
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Warte auf valide IMU Daten (Quaternion ist 0)...");
            return; // ABBRUCH: Nichts publishen!
        }

        // --- 2. Sicherheitscheck: Gyro/Accel ---
        if (std::isnan(imu_data.gyroscope()[0]) || std::isnan(imu_data.accelerometer()[0])) {
            return; // ABBRUCH: Keine NaNs publishen!
        }

        auto stamp = this->get_clock()->now();
        std::string frame_id = "imu_link";

        // --- Ab hier ist alles sicher -> Publishen ---

        // Accel
        geometry_msgs::msg::Vector3Stamped accel_msg;
        accel_msg.header.stamp = stamp;
        accel_msg.header.frame_id = frame_id;
        accel_msg.vector.x = imu_data.accelerometer()[0];
        accel_msg.vector.y = imu_data.accelerometer()[1];
        accel_msg.vector.z = imu_data.accelerometer()[2];
        pub_accel_->publish(accel_msg);

        // Gyro
        geometry_msgs::msg::Vector3Stamped gyro_msg;
        gyro_msg.header.stamp = stamp;
        gyro_msg.header.frame_id = frame_id;
        gyro_msg.vector.x = imu_data.gyroscope()[0];
        gyro_msg.vector.y = imu_data.gyroscope()[1];
        gyro_msg.vector.z = imu_data.gyroscope()[2];
        pub_gyro_->publish(gyro_msg);

        // Kombiniertes IMU Topic
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame_id;

        imu_msg.orientation.w = qw;
        imu_msg.orientation.x = qx;
        imu_msg.orientation.y = qy;
        imu_msg.orientation.z = qz;

        imu_msg.angular_velocity.x = imu_data.gyroscope()[0];
        imu_msg.angular_velocity.y = imu_data.gyroscope()[1];
        imu_msg.angular_velocity.z = imu_data.gyroscope()[2];

        imu_msg.linear_acceleration.x = imu_data.accelerometer()[0];
        imu_msg.linear_acceleration.y = imu_data.accelerometer()[1];
        imu_msg.linear_acceleration.z = imu_data.accelerometer()[2];

        // WICHTIG: Covariances auf 0 lassen, aber NICHT -1, da kiss_icp manchmal strikt ist.
        // 0.0 bedeutet "keine Varianz bekannt".
        imu_msg.orientation_covariance[0] = 0.0;
        imu_msg.angular_velocity_covariance[0] = 0.0;
        imu_msg.linear_acceleration_covariance[0] = 0.0;

        pub_imu_combined_->publish(imu_msg);
    }

    // ROS Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_combined_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_accel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_gyro_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;

    // Unitree Subscriber
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>> low_state_sub_;
};

int main(int argc, char **argv)
{
    // Netzwerk-Interface prüfen (z.B. eno1, eth0)
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        return -1;
    }

    // 1. Unitree SDK initialisieren (WICHTIG!)
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // 2. ROS2 initialisieren
    rclcpp::init(argc, argv);

    // 3. Node starten
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
