#include "asio.hpp"
#include <array>
#include <iostream>
#include <string>

class receiver {
public:
	receiver(asio::io_context& io_context,
	         const asio::ip::address& listen_address,
	         const asio::ip::address& multicast_address,
	         const short multicast_port)
	    : socket_(io_context) {
		// Create the socket so that multiple may be bound to the same address.
		asio::ip::udp::endpoint listen_endpoint(listen_address, multicast_port);
		socket_.open(listen_endpoint.protocol());
		socket_.set_option(asio::ip::udp::socket::reuse_address(true));
		socket_.bind(listen_endpoint);

		// Join the multicast group.
		socket_.set_option(asio::ip::multicast::join_group(multicast_address));

		do_receive();
	}

private:
	void do_receive() {
		socket_.async_receive_from(
		    asio::buffer(data_), sender_endpoint_,
		    [this](std::error_code ec, std::size_t length) {
			    if (!ec) {
				    std::cout.write(data_.data(), length);
				    std::cout << std::endl;

				    do_receive();
			    }
		    });
	}

	asio::ip::udp::socket socket_;
	asio::ip::udp::endpoint sender_endpoint_;
	std::array<char, 1024> data_;
};

int main(int argc, char* argv[]) {
	if (argc != 4) {
		std::cerr << "Usage: receiver <listen_address> <multicast_address> "
		             "<multicast_port>\n";
		return 1;
	}

	asio::io_context io_context;
	receiver r(io_context, asio::ip::make_address(argv[1]),
	           asio::ip::make_address(argv[2]), std::stoi(argv[3]));
	io_context.run();

	return 0;
}
