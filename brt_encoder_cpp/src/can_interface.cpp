#include "brt_encoder_cpp/can_utils.hpp"
#include <unistd.h>
#include <cstring>

namespace brt_encoder_cpp {

CANInterface::CANInterface(const std::string &interface) {
    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        throw std::runtime_error("Error while opening CAN socket");
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_);
        throw std::runtime_error("Error getting CAN interface index");
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(socket_);
        throw std::runtime_error("Error binding CAN socket");
    }
}

CANInterface::~CANInterface() {
    if (socket_ >= 0) {
        close(socket_);
    }
}

void CANInterface::sendFrame(const struct can_frame &frame) {
    if (write(socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("Error writing CAN frame");
    }
}

bool CANInterface::receiveFrame(struct can_frame &frame, int timeout_ms) {
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(socket_, &readSet);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(socket_ + 1, &readSet, nullptr, nullptr, &timeout);
    if (ret <= 0) {
        return false; // Timeout or error
    }

    if (read(socket_, &frame, sizeof(frame)) < 0) {
        throw std::runtime_error("Error reading CAN frame");
    }

    return true;
}

} // namespace brt_encoder_cpp