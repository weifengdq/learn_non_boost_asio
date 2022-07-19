#include <asio.hpp>
#include <iostream>

class udps {
public:
	udps(asio::io_context& io_context, const std::string& port)
	    : socket_(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(),
	                                                  std::stoi(port))) {
		do_receive();
	}
	~udps() { std::cout << "~udps()" << std::endl; }
	// void send(const std::string &msg)
	// {
	//     asio::async_write(socket_, asio::buffer(msg), [this](asio::error_code
	//     ec, std::size_t) {
	//         if (!ec) {
	//             do_receive();
	//         }
	//     });
	// }
	void do_receive() {
		socket_.async_receive(asio::buffer(recv_buffer_),
		                      [this](asio::error_code ec, std::size_t) {
			                      if (!ec) {
				                      std::cout << &recv_buffer_[0]
				                                << std::endl;
				                      do_receive();
			                      }
		                      });
	}

private:
	asio::ip::udp::socket socket_;
	char recv_buffer_[1024];
};

int main(int argc, const char** argv) {
	asio::io_context io_context;
	udps server(io_context, "8080");
	io_context.run();
	return 0;
}