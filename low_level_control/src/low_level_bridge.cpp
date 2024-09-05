#include "low_level_control/low_level_bridge.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <thread>  // Added for std::this_thread::sleep_for

// Function to unpack a float from 4 bytes
float unpackFloat(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    uint8_t data[4] = {byte1, byte2, byte3, byte4};
    float value;
    std::memcpy(&value, data, sizeof(float));
    return value;
}

// Function to pack a float into an array of bytes
void packFloatIntoArray(std::array<uint8_t, SIZE_OF_TX_DATA>& data, float value, size_t offset) {
    if (offset + sizeof(float) <= data.size()) {
        std::memcpy(&data[offset], &value, sizeof(float));
    }
}

OdometryProcessor::OdometryProcessor()
    : Node("odometry_processor"), serial_(io_), serial_port_(SERIAL_PORT), baud_rate_(BAUD_RATE), latest_odom_(nullptr),
      last_odom_feedback_time_(this->now()), queue_size_(0), timeout_count_(0) {

    // Setup ROS publishers and subscribers
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(PUBLISHED_ODOMETRY_TOPIC_NAME, 10);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(SUBSCRIBED_TWIST_TOPIC_NAME, 10, std::bind(&OdometryProcessor::twistCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(SUBSCRIBED_ODOMETRY_TOPIC_NAME, 10, std::bind(&OdometryProcessor::odometryCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdometryProcessor::publishOdometryAndTF, this));

    // Setup serial port
    try {
        serial_.open(serial_port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    } catch (boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        return;
    }

    // Start the serial communication thread
    serial_thread_ = boost::thread(boost::bind(&OdometryProcessor::serialThread, this));
}

OdometryProcessor::~OdometryProcessor() {
    io_.stop();
    serial_thread_.join();
    serial_.close();
    delete latest_odom_;  // Free the allocated memory for the latest odometry message
}

bool OdometryProcessor::enqueueMessage(const SerialMessage_t& msg) {
    if (message_queue_.push(msg)) {
        ++queue_size_;  // Increment the queue size when a message is enqueued
        return true;
    }

    // Handle the case where the queue is full
    SerialMessage_t discarded_msg;
    if (message_queue_.pop(discarded_msg)) {
        --queue_size_;  // Decrement the queue size when a message is dequeued
        if (message_queue_.push(msg)) {
            ++queue_size_;  // Increment again since we're adding a new message
            return true;
        }
    }

    RCLCPP_WARN(this->get_logger(), "Failed to enqueue message.");
    return false;
}

void OdometryProcessor::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    SerialMessage_t serial_msg;
    serial_msg.type = CMD_VEL;

    packFloatIntoArray(serial_msg.data, msg->linear.x, 0);
    packFloatIntoArray(serial_msg.data, msg->linear.y, sizeof(float));
    packFloatIntoArray(serial_msg.data, msg->angular.z, 2 * sizeof(float));

    if (!enqueueMessage(serial_msg)) {
        RCLCPP_WARN(this->get_logger(), "Unable to enqueue CMD_VEL message.");
    }
}

void OdometryProcessor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    rclcpp::Time current_time = this->now();

    // Check if the time since the last feedback is greater than the defined interval
    if ((current_time - last_odom_feedback_time_) < DT_BETWEEN_ODOM_FEEDBACK) {
        return;  // Skip this callback if the interval hasn't passed
    }

    // Update the last feedback time
    last_odom_feedback_time_ = current_time;

    SerialMessage_t serial_msg;
    serial_msg.type = ODOMETRY;

    // Convert quaternion to Euler (yaw)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Pack position
    packFloatIntoArray(serial_msg.data, msg->pose.pose.position.x, 0);
    packFloatIntoArray(serial_msg.data, msg->pose.pose.position.y, 4);
    packFloatIntoArray(serial_msg.data, static_cast<float>(yaw), 8); // Packing yaw

    // Pack velocity
    packFloatIntoArray(serial_msg.data, msg->twist.twist.linear.x, 12);
    packFloatIntoArray(serial_msg.data, msg->twist.twist.linear.y, 16);
    packFloatIntoArray(serial_msg.data, msg->twist.twist.angular.z, 20);

    // Pack covariance
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[0], 24);  // position_error.x
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[7], 28);  // position_error.y
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[35], 32); // position_error.angular
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[0], 36);  // velocity_error.x
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[7], 40);  // velocity_error.y
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[35], 44); // velocity_error.angular

    if (!enqueueMessage(serial_msg)) {
        RCLCPP_WARN(this->get_logger(), "Unable to enqueue ODOMETRY message.");
    }
}

void OdometryProcessor::serialThread() {
    std::vector<uint8_t> read_buffer(SIZE_OF_RX_DATA, 0);
    rclcpp::Rate rate(200);  // 200 Hz
    SerialMessage_t last_msg;  // Store the last message

    while (rclcpp::ok()) {
        SerialMessage_t msg;
        if (queue_size_ > 1) {
            if (message_queue_.pop(msg)) {
                --queue_size_;  // Decrement the queue size when popping a message
                last_msg = msg;  // Update the last message when popping a new one
                RCLCPP_DEBUG(this->get_logger(), "Popped message from queue and sending it.");
                sendSerialData(msg);
            }
        } else if (queue_size_ == 1) {
            RCLCPP_DEBUG(this->get_logger(), "Queue size is 1, resending the last message.");
            sendSerialData(last_msg);  // Keep sending the last message
        }

        // Handle incoming data with timeout
        boost::system::error_code ec;
        size_t len = 0;
        boost::asio::steady_timer timer(io_);
        bool timeout_occurred = false;

        // Set the timeout for 0.1 seconds (100 milliseconds)
        timer.expires_after(SERIAL_TIMEOUT_DURATION);
        timer.async_wait([&](const boost::system::error_code& error) {
            if (!error) {
                timeout_occurred = true;
                RCLCPP_DEBUG(this->get_logger(), "Timeout occurred, cancelling the read operation.");
                serial_.cancel();  // Cancel the read operation if timeout occurs
            }
        });

        // Start the asynchronous read operation
        boost::asio::async_read(serial_, boost::asio::buffer(read_buffer), boost::asio::transfer_at_least(SIZE_OF_RX_DATA),
                                [&](const boost::system::error_code& error, std::size_t bytes_transferred) {
                                    ec = error;
                                    len = bytes_transferred;
                                    RCLCPP_DEBUG(this->get_logger(), "Read operation completed. Bytes transferred: %zu", len);
                                    timer.cancel();  // Cancel the timer since data was read successfully
                                });

        // Run the IO context to execute the asynchronous operations
        io_.run();
        io_.reset();

        // Check if the read was successful or if a timeout occurred
        if (timeout_occurred) {
            RCLCPP_WARN(this->get_logger(), "Timeout occurred while reading from serial.");
            reconnectSerial();
        } 
        
        else {
            if (len > 0) {
                // RCLCPP_DEBUG(this->get_logger(), "Processing received data.");
                processReceivedData(read_buffer);
            } else if (ec) {
                RCLCPP_WARN(this->get_logger(), "Error reading from serial: %s", ec.message().c_str());
            }
        }

        rate.sleep();  // Control the thread frequency to 200 Hz
    }
}

void OdometryProcessor::sendSerialData(const SerialMessage_t& msg) {
    std::array<uint8_t, SIZE_OF_TX_DATA> buffer;

    // Set headers
    buffer[0] = HEADER;
    buffer[1] = HEADER;

    // Set message type indicator
    buffer[2] = static_cast<uint8_t>(msg.type);

    // Copy data from SerialMessage into the buffer
    std::memcpy(&buffer[3], msg.data.data(), SIZE_OF_TX_DATA - 5);  // SIZE_OF_TX_DATA - 5 to account for header, type, checksum, and tail

    // Compute checksum
    uint8_t checksum = 0;
    for (size_t i = 3; i < SIZE_OF_TX_DATA - 2; ++i) {
        checksum += buffer[i];
    }

    // Append checksum and tail
    buffer[SIZE_OF_TX_DATA - 2] = checksum;
    buffer[SIZE_OF_TX_DATA - 1] = TAIL;

    // Log the buffer before sending
    std::stringstream ss;
    for (const auto& byte : buffer) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    RCLCPP_DEBUG(this->get_logger(), "Sending serial data: %s", ss.str().c_str());

    // Send data over serial port using boost::asio
    boost::asio::write(serial_, boost::asio::buffer(buffer, SIZE_OF_TX_DATA));
    RCLCPP_DEBUG(this->get_logger(), "Serial data sent.");
}

void OdometryProcessor::processReceivedData(const std::vector<uint8_t>& buffer) {
    RCLCPP_DEBUG(this->get_logger(), "Processing received serial data.");
    if (buffer[0] == HEADER && buffer[1] == HEADER) {
        uint8_t checksum = 0;
        for (size_t i = 2; i < SIZE_OF_RX_DATA - 2; i++) {
            checksum += buffer[i];
        }
        uint8_t message_checksum = buffer[SIZE_OF_RX_DATA - 2];
        if (checksum == message_checksum) {
            RCLCPP_INFO(this->get_logger(), "Message received successfully");

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->now();
            odom.header.frame_id = "odom";  // Set parent frame ID to "odom"
            odom.child_frame_id = "base_link";  // Set child frame ID to "base_link"

            float temp_float;

            // Unpacking position
            temp_float = unpackFloat(buffer[2], buffer[3], buffer[4], buffer[5]);
            odom.pose.pose.position.x = temp_float;
            temp_float = unpackFloat(buffer[6], buffer[7], buffer[8], buffer[9]);
            odom.pose.pose.position.y = temp_float;
            temp_float = unpackFloat(buffer[10], buffer[11], buffer[12], buffer[13]);

            // Convert yaw (Euler angle) to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, temp_float);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            // Unpacking velocity
            temp_float = unpackFloat(buffer[14], buffer[15], buffer[16], buffer[17]);
            odom.twist.twist.linear.x = temp_float;
            temp_float = unpackFloat(buffer[18], buffer[19], buffer[20], buffer[21]);
            odom.twist.twist.linear.y = temp_float;
            temp_float = unpackFloat(buffer[22], buffer[23], buffer[24], buffer[25]);
            odom.twist.twist.angular.z = temp_float;

            // Unpacking position error covariance
            temp_float = unpackFloat(buffer[26], buffer[27], buffer[28], buffer[29]);
            odom.pose.covariance[0] = temp_float;
            temp_float = unpackFloat(buffer[30], buffer[31], buffer[32], buffer[33]);
            odom.pose.covariance[7] = temp_float;
            temp_float = unpackFloat(buffer[34], buffer[35], buffer[36], buffer[37]);
            odom.pose.covariance[35] = temp_float;

            // Unpacking velocity error covariance
            temp_float = unpackFloat(buffer[38], buffer[39], buffer[40], buffer[41]);
            odom.twist.covariance[0] = temp_float;
            temp_float = unpackFloat(buffer[42], buffer[43], buffer[44], buffer[45]);
            odom.twist.covariance[7] = temp_float;
            temp_float = unpackFloat(buffer[46], buffer[47], buffer[48], buffer[49]);
            odom.twist.covariance[35] = temp_float;

            delete latest_odom_;  // Free the old odometry message
            latest_odom_ = new nav_msgs::msg::Odometry(odom);  // Store the latest odometry message
            RCLCPP_DEBUG(this->get_logger(), "Odometry message unpacked and stored.");

            // Publish TF for each wheel based on the odometry data
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch in received serial data. Expected: %02x, Calculated: %02x. Buffer: ", buffer[SIZE_OF_RX_DATA - 2], checksum);
            for (const auto& byte : buffer) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            std::cout << std::endl;

            // clearSerialBuffer();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Header or Tail mismatch in received serial data. Buffer: ");
        for (const auto& byte : buffer) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;

        // clearSerialBuffer();
    }
}

void OdometryProcessor::reconnectSerial() {
    RCLCPP_WARN(this->get_logger(), "Attempting to reconnect to the serial port...");
    serial_.close();
    std::this_thread::sleep_for(SERIAL_RECONNECT_DELAY);  // Sleep before reconnecting
    try {
        serial_.open(serial_port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        RCLCPP_INFO(this->get_logger(), "Serial port reconnected successfully.");
    } catch (boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect to serial port: %s", e.what());
    }
}

void OdometryProcessor::publishOdometryAndTF() {
    if (latest_odom_) {
        latest_odom_->header.stamp = this->now();  // Update the timestamp
        odometry_pub_->publish(*latest_odom_);
        publishWheelTFs(*latest_odom_);
        // publishTF(*latest_odom_);
    }
}

void OdometryProcessor::publishTF(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = odom.header.stamp;
    odom_tf.header.frame_id = odom.header.frame_id;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
}

void OdometryProcessor::publishWheelTFs(const nav_msgs::msg::Odometry& odom) {
    // Constants
    const float wheel_radius = WHEEL_DIAMETER / 2.0;
    const float lx_ly_sum = VEHICLE_WIDTH + VEHICLE_LENGTH;

    // Get velocities from odometry
    float vx = odom.twist.twist.linear.x;
    float vy = odom.twist.twist.linear.y;
    float wz = odom.twist.twist.angular.z;

    // Calculate wheel angular velocities
    float v_left_front = (1.0 / wheel_radius) * (vx - vy - lx_ly_sum * wz);
    float v_right_front = (1.0 / wheel_radius) * (vx + vy + lx_ly_sum * wz);
    float v_left_rear = (1.0 / wheel_radius) * (vx + vy - lx_ly_sum * wz);
    float v_right_rear = (1.0 / wheel_radius) * (vx - vy + lx_ly_sum * wz);

    // Publish TF for each wheel
    auto publish_wheel_tf = [&](const std::string& wheel_name, float wheel_velocity) {
        geometry_msgs::msg::TransformStamped wheel_tf;
        wheel_tf.header.stamp = odom.header.stamp;
        wheel_tf.header.frame_id = "base_link";
        wheel_tf.child_frame_id = wheel_name;

        // Assuming the wheel rotation is around the Y-axis
        wheel_tf.transform.translation.x = 0.0;
        wheel_tf.transform.translation.y = 0.0;
        wheel_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, wheel_velocity, 0.0);  // Wheel rotation around the Y-axis
        wheel_tf.transform.rotation.x = q.x();
        wheel_tf.transform.rotation.y = q.y();
        wheel_tf.transform.rotation.z = q.z();
        wheel_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(wheel_tf);
    };

    publish_wheel_tf("left_front_wheel", v_left_front);
    publish_wheel_tf("right_front_wheel", v_right_front);
    publish_wheel_tf("left_rear_wheel", v_left_rear);
    publish_wheel_tf("right_rear_wheel", v_right_rear);
}

void OdometryProcessor::clearSerialBuffer() {
    std::vector<uint8_t> dummy_buffer(1024);
    boost::system::error_code ec;
    while (serial_.read_some(boost::asio::buffer(dummy_buffer), ec)) {
        if (ec) {
            break;
        }
    }
}

void OdometryProcessor::spin() {
    rclcpp::Rate rate(200);  // 200 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<OdometryProcessor>();
    processor->spin();
    rclcpp::shutdown();
    return 0;
}
