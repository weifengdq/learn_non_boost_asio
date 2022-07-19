#include "asio.hpp"
#include <iostream>

class unix_socket_tx {
public:
	unix_socket_tx(asio::io_context& io_context, const std::string& path)
	    : endpoint_(asio::local::stream_protocol::endpoint(path))
	    , socket_(io_context)
	    , timer_(io_context) {
		do_connect();
	}

private:
	void do_connect() {
		socket_.async_connect(endpoint_, [this](std::error_code ec) {
			if (!ec) {
				do_write();
			} else {
				std::cout << "Error 1: " << ec.message() << "\n";
			}
		});
	}
	void do_write() {
		socket_.async_write_some(
		    asio::buffer(data_),
		    [this](std::error_code ec, std::size_t /*size*/) {
			    if (!ec) {
				    do_timeout();
			    } else {
				    std::cout << "Error 2: " << ec.message() << "\n";
			    }
		    });
	}
	void do_timeout() {
		timer_.expires_after(std::chrono::seconds(1));
		timer_.async_wait([this](std::error_code ec) {
			if (!ec) {
				do_write();
			} else {
				std::cout << "Error 3: " << ec.message() << "\n";
			}
		});
	}
	asio::local::stream_protocol::endpoint endpoint_;
	asio::local::stream_protocol::socket socket_;
	asio::steady_timer timer_;
	std::array<char, 1024> data_{'a', 'b', 'c', 'd', 'e'};
};

int main(int argc, const char** argv) {
	try {
		if (argc != 2) {
			std::cerr << "Usage: " << argv[0] << " <path>\n";
			return 1;
		}
		asio::io_context io_context;
		unix_socket_tx s(io_context, argv[1]);
		io_context.run();
	} catch (std::exception& e) {
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}