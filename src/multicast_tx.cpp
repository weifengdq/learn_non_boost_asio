#include "asio.hpp"
#include <iostream>
#include <sstream>
#include <string>

class multicast_tx {
public:
	multicast_tx(asio::io_context& io_context,
	             const asio::ip::address& multicast_address,
	             const short multicast_port)
	    : endpoint_(multicast_address, multicast_port)
	    , socket_(io_context, endpoint_.protocol())
	    , timer_(io_context) {
		do_send();
	}

private:
	void do_send() {
		socket_.async_send_to(asio::buffer(data_), endpoint_,
		                      [this](std::error_code ec, std::size_t len) {
			                      if (!ec) {
				                      do_timeout();
			                      }
		                      });
	}

	void do_timeout() {
		timer_.expires_after(std::chrono::seconds(1));
		timer_.async_wait([this](std::error_code ec) {
			if (!ec) {
				do_send();
			}
		});
	}

	asio::ip::udp::socket socket_;
	asio::ip::udp::endpoint endpoint_;
	asio::steady_timer timer_;
	std::array<char, 1024> data_{'a', 'b', 'c', 'd', 'e'};
};

int main(int argc, const char** argv) {
	if (argc != 3) {
		std::cerr
		    << "Usage: multicast_tx <multicast_address> <multicast_port>\n";
		return 1;
	}
	asio::io_context io_context;
	multicast_tx s(io_context, asio::ip::make_address(argv[1]),
	               std::stoi(argv[2]));
	io_context.run();
	return 0;
}