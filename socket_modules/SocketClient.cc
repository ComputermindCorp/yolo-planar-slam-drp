#include "SocketClient.h"

#include <iostream>

namespace socket_publisher {

SocketClient::SocketClient(const std::string& server_uri)
    : client_(), callback_() {
    // register socket callbacks
    client_.set_open_listener(std::bind(&SocketClient::on_open, this));
    client_.set_close_listener(std::bind(&SocketClient::on_close, this));
    client_.set_fail_listener(std::bind(&SocketClient::on_fail, this));

    // start connection
    client_.connect(server_uri);
    // get socket
    socket_ = client_.socket();

    socket_->on("signal", std::bind(&SocketClient::on_receive, this, std::placeholders::_1));
}

void SocketClient::on_close() {
    std::cerr << "connection closed correctly" << std::endl;
}

void SocketClient::on_fail() {
    std::cerr << "connection closed incorrectly" << std::endl;
}

void SocketClient::on_open() {
    std::cerr << "connection to server" << std::endl;
}

void SocketClient::on_receive(const sio::event& event) {
    try {
        const std::string message = event.get_message()->get_string();
        if (callback_) {
            callback_(message);
        }
    }
    catch (std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
}

} // namespace socket_publisher
