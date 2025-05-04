#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

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

// --------- 全局可配置变量 ---------
const std::string INPUT_SOURCE = "10.mp4";              // 输入方式："0"为摄像头，或替换为视频文件路径
const int FRAME_INTERVAL_MS = 33;                  // 帧间隔时间（毫秒）

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

void setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void sendChunks(int client_sock, const std::vector<unsigned char>& buf, uint32_t& data_id) {
    uint32_t total = buf.size();
    uint32_t offset = 0;
    while (offset < total) {
        uint32_t chunk = std::min<uint32_t>(MAX_DATA_SIZE, total - offset);
        MessageBuffer msg{};
        msg.Start           = htons(START_SYMBOL);
        msg.MessageType     = htons(IMAGE_MSG);
        msg.DataID          = htonl(data_id);
        msg.DataTotalLength = htonl(total);
        msg.Offset          = htonl(offset);
        msg.DataLength      = htonl(chunk);
        memcpy(msg.Data, buf.data() + offset, chunk);
        msg.End             = htons(END_SYMBOL);
        size_t to_send = sizeof(MessageBuffer) - MAX_DATA_SIZE + chunk;
        send(client_sock, reinterpret_cast<char*>(&msg), to_send, MSG_NOSIGNAL);
        offset += chunk;
    }
    ++data_id;
}

int main() {
    std::string input = INPUT_SOURCE;

    std::cout << "[服务端] 启动成功，正在绑定端口..." << std::endl;

    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("服务端 socket 创建失败");
        return -1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8888);

    if (bind(server_sock, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("服务端绑定失败");
        return -1;
    }
    std::cout << "[服务端] 绑定成功，监听中 (端口 8888)..." << std::endl;

    listen(server_sock, 1);
    std::cout << "[服务端] 待机中，等待客户端连接..." << std::endl;

    int client_sock = accept(server_sock, nullptr, nullptr);
    if (client_sock < 0) {
        perror("accept 失败");
        return -1;
    }
    std::cout << "[服务端] 客户端已连接！开始传输视频数据..." << std::endl;

    setNonBlocking(client_sock);

    cv::VideoCapture cap;
    if (input == "0") {
        cap.open(0); // 摄像头输入
    } else {
        cap.open(input, cv::CAP_FFMPEG); // 使用 FFmpeg 打开 MP4 文件
    }

    if (!cap.isOpened()) {
        std::cerr << "[服务端] 视频文件或摄像头打开失败！" << std::endl;
        return -1;
    }

    uint32_t data_id = 0;
    std::vector<unsigned char> buf;
    bool running = true;

    while (running) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "[服务端] 读取视频帧失败。" << std::endl;
            break;
        }
        cv::imencode(".jpg", frame, buf);
        sendChunks(client_sock, buf, data_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(FRAME_INTERVAL_MS));
    }

    std::string end_msg = "__STREAM_END__";
    MessageBuffer m{};
    m.Start           = htons(START_SYMBOL);
    m.MessageType     = htons(STRING_MSG);
    m.DataID          = htonl(data_id++);
    m.DataTotalLength = htonl(end_msg.size());
    m.Offset          = htonl(0);
    m.DataLength      = htonl(end_msg.size());
    memcpy(m.Data, end_msg.data(), end_msg.size());
    m.End             = htons(END_SYMBOL);
    send(client_sock, reinterpret_cast<char*>(&m), sizeof(MessageBuffer) - MAX_DATA_SIZE + end_msg.size(), MSG_NOSIGNAL);

    close(client_sock);
    close(server_sock);
    std::cout << "[服务端] 视频传输结束，服务关闭。" << std::endl;
    return 0;
}
