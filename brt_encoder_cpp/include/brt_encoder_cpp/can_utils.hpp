#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string>
#include <stdexcept>

namespace brt_encoder_cpp {

class CANInterface {
public:
    explicit CANInterface(const std::string &interface);
    ~CANInterface();
    
    void sendFrame(const struct can_frame &frame);
    bool receiveFrame(struct can_frame &frame, int timeout_ms = 100);
    
private:
    int socket_;
};

} // namespace brt_encoder_cpp