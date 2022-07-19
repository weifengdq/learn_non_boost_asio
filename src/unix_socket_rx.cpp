#include "asio.hpp"
#include <iostream>

class unix_socket_rx {
public:
	unix_socket_rx(asio::io_context& io_context, const std::string& path)
	    : endpoint_(asio::local::stream_protocol::endpoint(path))
	    , acceptor_(io_context, endpoint_)
	    , socket_(io_context) {
		// unlink(path.c_str());
		do_accept();
	}

private:
	void do_accept() {
		acceptor_.async_accept(socket_, [this](std::error_code ec) {
			if (!ec) {
				do_read();
			} else {
				std::cout << "Error 1: " << ec.message() << "\n";
			}
		});
	}
	void do_read() {
		socket_.async_read_some(
		    asio::buffer(data_), [this](std::error_code ec, std::size_t len) {
			    if (!ec) {
				    std::cout << std::string(data_.data(), len)
				              << " len:" << len << "\n";
				    do_read();
			    } else {
				    std::cout << "Error 2: " << ec.message() << "\n";
			    }
		    });
	}
	asio::local::stream_protocol::endpoint endpoint_;
	asio::local::stream_protocol::acceptor acceptor_;
	asio::local::stream_protocol::socket socket_;
	std::array<char, 1024> data_;
};

int main(int argc, const char** argv) {
	if (argc != 2) {
		std::cerr << "Usage: " << argv[0] << " <path>\n";
		return 1;
	}
	asio::io_context io_context;
	std::remove(argv[1]); // existing file is removed
	unix_socket_rx s(io_context, argv[1]);
	io_context.run();
	return 0;
}