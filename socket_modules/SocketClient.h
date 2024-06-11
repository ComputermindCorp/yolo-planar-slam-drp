#ifndef SOCKET_VIEWER_SOCKET_CLIENT_H
#define SOCKET_VIEWER_SOCKET_CLIENT_H

#include <sioclient/sio_client.h>

namespace socket_publisher {

class SocketClient {
public:
    SocketClient(const std::string& server_uri);

    void emit(const std::string tag, const std::string buffer) {
        socket_->emit(tag, buffer);
    }

    void set_signal_callback(std::function<void(std::string)> callback) {
        callback_ = callback;
    }

private:
    void on_close();
    void on_fail();
    void on_open();
    void on_receive(const sio::event& event);

    sio::client client_;
    sio::socket::ptr socket_;

    std::function<void(std::string)> callback_;
};

} // namespace socket_publisher

#endif // SOCKET_VIEWER_SOCKET_CLIENT_H
