#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

using namespace std::chrono;

int main() {
    int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        std::cerr << "Không thể mở cổng serial\n";
        return 1;
    }

    // Cấu hình serial
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    tcgetattr(serial_port, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(serial_port, TCSANOW, &tty);

    // Biến xử lý frame
    uint8_t data;
    uint8_t _rxBuffer[14];
    int _rxIndex = 0;

    // Đo thời gian giữa các frame hợp lệ
    auto last_frame_time = high_resolution_clock::now();

    while (true) {
        if (read(serial_port, &data, 1) > 0) {
            if (_rxIndex == 0) {
                if (data == 0x2A) {
                    _rxBuffer[_rxIndex++] = data;
                } else {
                    _rxIndex = 0;
                }
            } else {
                _rxBuffer[_rxIndex++] = data;

                if (_rxIndex == 14) {
                    if (_rxBuffer[13] == 0x23) {
                        // ✅ Frame hợp lệ

                        if (_rxBuffer[1] == 0x03){
                            std::cout << _rxBuffer[1] << std::endl;
                            auto now = high_resolution_clock::now();
                            auto duration = duration_cast<milliseconds>(now - last_frame_time);
                            last_frame_time = now;

                            std::cout << "Frame hợp lệ (sau " << duration.count() << " ms): ";
                            for (int i = 0; i < 14; i++) {
                                printf("%02X ", _rxBuffer[i]);
                            }
                            std::cout << std::endl;
                        }
                    } else {
                        std::cout << "Sai footer\n";
                    }
                    _rxIndex = 0;
                }
            }
        }
    }

    close(serial_port);
    return 0;
}