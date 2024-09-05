#ifndef LOW_LEVEL_BRIDGE_HPP
#define LOW_LEVEL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <array>
#include <atomic>
#include <chrono>

// Serial communication configs
#define SERIAL_PORT "/dev/ttyUSB1"
#define BAUD_RATE 115200
#define HEADER 0xC8
#define TAIL 0xC7
#define SIZE_OF_RX_DATA 52
#define SIZE_OF_TX_DATA 53
#define QUEUE_CAPACITY 10
#define DT_BETWEEN_ODOM_FEEDBACK std::chrono::seconds(5)  // Update once every 5 seconds
#define SERIAL_TIMEOUT_DURATION std::chrono::milliseconds(100)  // Timeout duration of 0.1 seconds
#define SERIAL_RECONNECT_DELAY std::chrono::milliseconds(200)  // Reconnect delay of 2 seconds

// ROS configs
#define PUBLISHED_ODOMETRY_TOPIC_NAME "odometry/wheel_encoders"
#define SUBSCRIBED_TWIST_TOPIC_NAME "cmd_vel"
#define SUBSCRIBED_ODOMETRY_TOPIC_NAME "odometry/filtered"
#define PARENT_FRAME_ID "base_link"


// tf configs
#define WHEEL_DIAMETER 0.08  // 80 mm
#define VEHICLE_WIDTH 0.416 // meter
#define VEHICLE_LENGTH 0.255 //meter

typedef enum {
    CMD_VEL = 1,
    ODOMETRY = 2
} MessageType_e;

typedef struct {
    MessageType_e type;
    std::array<uint8_t, SIZE_OF_TX_DATA> data;
} SerialMessage_t;

class OdometryProcessor : public rclcpp::Node {
public:
    OdometryProcessor();
    ~OdometryProcessor();
    void spin();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void serialThread();
    void sendSerialData(const SerialMessage_t& msg);
    void processReceivedData(const std::vector<uint8_t>& buffer);
    bool enqueueMessage(const SerialMessage_t& msg);
    void clearSerialBuffer();
    void publishTF(const nav_msgs::msg::Odometry& odom);
    void publishOdometryAndTF();
    void reconnectSerial();

    // New function to publish TF based on wheel velocities
    void publishWheelTFs(const nav_msgs::msg::Odometry& odom);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry* latest_odom_;  // Use a pointer to store the latest odometry message

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    boost::thread serial_thread_;
    std::string serial_port_;

    int baud_rate_;
    boost::lockfree::queue<SerialMessage_t> message_queue_{QUEUE_CAPACITY};
    
    std::atomic<size_t> queue_size_;  // Track the size of the queue
    rclcpp::Time last_odom_feedback_time_;  // Track last odometry feedback time
    std::atomic<int> timeout_count_;  // Track consecutive timeouts
};

// Function to unpack a float from 4 bytes
float unpackFloat(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

#endif // LOW_LEVEL_BRIDGE_HPP
