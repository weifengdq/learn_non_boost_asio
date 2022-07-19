#include <asio.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
	asio::io_context io_context;

	asio::serial_port s(io_context, "/dev/ttyUSB0", 115200);
	if (s.is_open()) {
		s.close();
		std::cout << "open" << std::endl;
	} else {
		std::cout << "close" << std::endl;
	}
	s.open("/dev/ttyUSB0");

	std::vector<char> frame;
	while (true) {
		char data[256];
		int n = s.read_some(asio::buffer(data));
		for (int i = 0; i < n; i++) {
			frame.push_back(data[i]);
			if (data[i] == '\n') {
				std::cout << "read: " << frame.size() << " ";
				for (int i = 0; i < frame.size(); i++) {
					std::cout << frame[i];
				}
				std::cout << std::endl;
				s.async_write_some(
				    asio::buffer(frame), [](const std::error_code& ec,
				                            std::size_t bytes_transferred) {
					    std::cout << "write: " << bytes_transferred
					              << std::endl;
				    });
				frame.clear();
			}
		}
	}
	s.close();
	return 0;
}
