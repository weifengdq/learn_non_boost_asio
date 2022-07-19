#include <asio.hpp>
#include <iostream>

class udpc {
public:
	udpc(asio::io_context& io_context, const std::string& host,
	     const std::string& port)
	    : socket_(io_context,
	              asio::ip::udp::endpoint(asio::ip::udp::v4(), std::stoi(port)))
	    , resolver_(io_context)
	    , host_(host)
	    , port_(port) {
		asio::ip::udp::resolver::query query(host_, port_);
		sender_endpoint_ = *resolver_.resolve(query);
		send("udpc begin receive");
	}
	~udpc() { std::cout << "~udpc()" << std::endl; }

	void send(const std::string& msg) {
		socket_.async_send_to(asio::buffer(msg), sender_endpoint_,
		                      [this](asio::error_code ec, std::size_t) {});
	}

private:
	asio::ip::udp::socket socket_;
	asio::ip::udp::resolver resolver_;
	asio::ip::udp::endpoint sender_endpoint_;
	std::string host_;
	std::string port_;
};

int main(int argc, const char** argv) {
	if (argc != 3) {
		std::cerr << "Usage: udpc <host> <port>\n";
		return 1;
	}
	asio::io_context io_context;
	udpc client(io_context, argv[1], argv[2]);
	client.send("second send");
	io_context.run();
	return 0;
}

// #include <iostream>
// #include <asio.hpp>

// class udpc
// {
// public:
//     udpc(asio::io_context &io_context, const std::string &host, const
//     std::string &port)
//         : socket_(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(),
//         std::stoi(port))),
//           resolver_(io_context),
//           host_(host),
//           port_(port)
//     {
//         asio::ip::udp::resolver::query query(host_, port_);
//         sender_endpoint_ = *resolver_.resolve(query);
//         send("udpc begin receive");
//         receive();
//     }
//     ~udpc()
//     {
//         std::cout << "~udpc()" << std::endl;
//     }

//     void send(const std::string &msg)
//     {
//         socket_.async_send_to(asio::buffer(msg), sender_endpoint_,
//         [this](asio::error_code ec, std::size_t) {});
//     }

//     void receive()
//     {
//         socket_.async_receive_from(asio::buffer(recv_buffer_),
//         sender_endpoint_, [this](asio::error_code ec, std::size_t len) {
//             if (!ec) {
//                 recv_buffer_[len] = '\0';
//                 std::cout << &recv_buffer_[0] << std::endl;
//                 receive();
//             }
//         });
//     }

// private:
//     asio::ip::udp::socket socket_;
//     asio::ip::udp::resolver resolver_;
//     asio::ip::udp::endpoint sender_endpoint_;
//     std::string host_;
//     std::string port_;
//     char recv_buffer_[1600];
// };

// int main(int argc, const char **argv)
// {
//     if (argc != 3)
//     {
//         std::cerr << "Usage: udpc <host> <port>\n";
//         return 1;
//     }
//     asio::io_context io_context;
//     udpc client(io_context, argv[1], argv[2]);
//     client.send("second send");
//     io_context.run();
//     return 0;
// }