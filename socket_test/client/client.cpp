#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

// 枚举消息类型
enum MessageType : uint16_t {
    STRING_MSG        = 0x0000,
    IMAGE_MSG         = 0x1145,
    CAMERA_INFO       = 0x1419,
    TRANSFORM         = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

static const uint16_t START_SYMBOL = 0x0D00;
static const uint16_t END_SYMBOL   = 0x0721;
static const size_t   MAX_DATA_SIZE = 10218;

#pragma pack(push,1)
struct MessageBuffer {
    uint16_t  Start;
    uint16_t  MessageType;
    uint32_t  DataID;
    uint32_t  DataTotalLength;
    uint32_t  Offset;
    uint32_t  DataLength;
    unsigned char Data[MAX_DATA_SIZE];
    uint16_t  End;
};
#pragma pack(pop)

struct Partial {
    uint32_t total;
    std::vector<unsigned char> buffer;
    uint32_t received;
};

void setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void handleIncoming(int sock, std::function<void(const cv::Mat&)> onFrame, std::function<void(const std::string&)> onAck) {
    std::map<uint32_t, Partial> store;
    while (true) {
        uint16_t start_net;
        int r = recv(sock, (char*)&start_net, 2, MSG_WAITALL);
        if (r <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        uint16_t type_net;
        recv(sock, (char*)&type_net, 2, MSG_WAITALL);

        uint32_t id_net, total_net, offset_net, length_net;
        recv(sock, (char*)&id_net, 4, MSG_WAITALL);
        recv(sock, (char*)&total_net, 4, MSG_WAITALL);
        recv(sock, (char*)&offset_net, 4, MSG_WAITALL);
        recv(sock, (char*)&length_net, 4, MSG_WAITALL);

        uint16_t start  = ntohs(start_net);
        if (start != START_SYMBOL) continue;
        uint16_t type   = ntohs(type_net);
        uint32_t id     = ntohl(id_net);
        uint32_t total  = ntohl(total_net);
        uint32_t offset = ntohl(offset_net);
        uint32_t length = ntohl(length_net);

        std::vector<unsigned char> data(length);
        recv(sock, (char*)data.data(), length, MSG_WAITALL);

        uint16_t end_net;
        recv(sock, (char*)&end_net, 2, MSG_WAITALL);
        uint16_t end = ntohs(end_net);
        if (end != END_SYMBOL) continue;

        if (type == IMAGE_MSG) {
            auto& p = store[id];
            if (p.buffer.empty()) {
                p.total = total;
                p.buffer.resize(total);
                p.received = 0;
            }
            memcpy(p.buffer.data() + offset, data.data(), length);
            p.received += length;
            if (p.received >= p.total) {
                cv::Mat frame = cv::imdecode(p.buffer, cv::IMREAD_COLOR);
                if (!frame.empty()) {
                    std::cout << "[客户端] 收到图像消息 ID: " << id << ", 尺寸: " << frame.cols << " x " << frame.rows << std::endl;
                    onFrame(frame);
                } else {
                    std::cerr << "[客户端] 图像解码失败！" << std::endl;
                }
                store.erase(id);
            }
        } else if (type == STRING_MSG) {
            std::string msg((char*)data.data(), length);
            std::cout << "[客户端] 收到字符串消息: " << msg << std::endl;
            if (msg == "__STREAM_END__") break;
            if (onAck) onAck(msg);
        }
    }
}

int main() {
    std::cout << "[客户端] 启动，尝试连接到服务器..." << std::endl;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("客户端 socket 创建失败");
        return -1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8888);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

    if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("连接服务器失败");
        return -1;
    }
    std::cout << "[客户端] 连接服务器成功，等待图像数据..." << std::endl;

    setNonBlocking(sock);

    handleIncoming(sock, [](const cv::Mat& frame){
        std::cout << "[客户端] 显示图像，尺寸: " << frame.cols << "x" << frame.rows << std::endl;
        cv::imshow("Stream", frame);
        cv::waitKey(1);
    }, [](const std::string& ack){
        printf("Ack: %s\n", ack.c_str());
    });

    std::cout << "[客户端] 图像接收完成，按任意键退出..." << std::endl;
    cv::waitKey(0);
    close(sock);
    return 0;
}
