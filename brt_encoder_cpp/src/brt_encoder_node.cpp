#include "brt_encoder_cpp/can_utils.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <linux/can.h>
#include <memory>

class BRTEncoderNode : public rclcpp::Node {
public:
    BRTEncoderNode() : Node("brt_encoder_node") {
        // 参数声明
        declare_parameters();
        
        // 初始化CAN接口
        init_can_interface();
        
        // 初始化发布器
        init_publishers();
        
        // 初始化定时器
        init_timer();
        
        RCLCPP_INFO(get_logger(), "BRT编码器节点已启动");
    }

private:
    void declare_parameters() {
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("encoder_id", 1);
        this->declare_parameter<double>("polling_rate", 10.0);
        this->declare_parameter<int>("resolution", 1024);
        this->declare_parameter<int>("max_turns", 16);
    }

    void init_can_interface() {
        try {
            can_interface_ = std::make_unique<brt_encoder_cpp::CANInterface>(
                this->get_parameter("can_interface").as_string());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "CAN接口初始化失败: %s", e.what());
            throw;
        }
    }

    void init_publishers() {
        pub_raw_ = create_publisher<std_msgs::msg::Float32>("encoder/raw", 10);
        pub_angle_ = create_publisher<std_msgs::msg::Float32>("encoder/angle", 10);
        pub_turns_ = create_publisher<std_msgs::msg::Float32>("encoder/turns", 10);
        pub_can_ = create_publisher<can_msgs::msg::Frame>("can_tx", 10);
    }

    void init_timer() {
        auto polling_rate = this->get_parameter("polling_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / polling_rate),
            std::bind(&BRTEncoderNode::poll_encoder, this));
    }

    void poll_encoder() {
        // 准备CAN帧
        struct can_frame frame;
        frame.can_id = this->get_parameter("encoder_id").as_int();
        frame.can_dlc = 4;
        frame.data[0] = 0x04; // 长度
        frame.data[1] = frame.can_id;
        frame.data[2] = 0x01; // 读取指令
        frame.data[3] = 0x00; // 数据

        try {
            // 发送CAN帧
            can_interface_->sendFrame(frame);
            
            // 发布到CAN话题(可选)
            publish_can_frame(frame, true);
            
            // 接收响应
            if (can_interface_->receiveFrame(frame)) {
                process_response(frame);
            } else {
                RCLCPP_WARN(get_logger(), "未收到编码器响应");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "通信错误: %s", e.what());
        }
    }

    void process_response(const struct can_frame &frame) {
        if (frame.can_dlc >= 7 && frame.data[1] == this->get_parameter("encoder_id").as_int()) {
            // 解析32位编码器值(小端序)
            uint32_t raw_value = (frame.data[6] << 24) | 
                               (frame.data[5] << 16) | 
                               (frame.data[4] << 8) | 
                               frame.data[3];
            
            // 计算角度和圈数
            int resolution = this->get_parameter("resolution").as_int();
            float angle = (raw_value % resolution) * 360.0f / resolution;
            float turns = static_cast<float>(raw_value) / resolution;
            
            // 发布消息
            auto angle_msg = std_msgs::msg::Float32();
            angle_msg.data = angle;
            pub_angle_->publish(angle_msg);
            
            auto turns_msg = std_msgs::msg::Float32();
            turns_msg.data = turns;
            pub_turns_->publish(turns_msg);
            
            auto raw_msg = std_msgs::msg::Float32();
            raw_msg.data = static_cast<float>(raw_value);
            pub_raw_->publish(raw_msg);
            
            // 发布接收到的CAN帧(可选)
            publish_can_frame(frame, false);
        }
    }

    void publish_can_frame(const struct can_frame &frame, bool is_tx) {
        auto msg = can_msgs::msg::Frame();
        msg.header.stamp = this->now();
        msg.id = frame.can_id;
        msg.dlc = frame.can_dlc;
        msg.is_extended = false;
        msg.is_rtr = false;
        msg.is_error = false;
        
        for (int i = 0; i < frame.can_dlc; ++i) {
            msg.data[i] = frame.data[i];
        }
        
        pub_can_->publish(msg);
    }

    std::unique_ptr<brt_encoder_cpp::CANInterface> can_interface_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_raw_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_turns_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BRTEncoderNode>());
    rclcpp::shutdown();
    return 0;
}