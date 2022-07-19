#include <asio.hpp>
#include <iostream>

class first {
public:
	first(asio::io_context& io_context)
	    : time_(io_context, std::chrono::seconds(2))
	    , count_{2} {
		time_.async_wait([this](asio::error_code ec) {
			if (!ec) {
				this->handle_wait();
			}
		});
	}
	~first() { std::cout << "~first()" << std::endl; }

	void handle_wait() { std::cout << count_ << std::endl; }

private:
	asio::steady_timer time_;
	int count_;
};

int main(int argc, const char** argv) {
	asio::io_context io_context;
	first f(io_context);
	io_context.run();
	return 0;
}

// void print(const asio::error_code &ec, int * n)
// {
//     std::cout << *n << std::endl;
// }

// int main()
// {
//     int count = 2;
//     asio::io_context io;
//     asio::steady_timer t(io, asio::chrono::seconds(2));
//     t.async_wait([&](const asio::error_code &ec) {
//         std::cout << "xxxx" << std::endl;
//         print(ec, &count);
//     });
//     std::cout << "print run after 2s..." << std::endl;
//     io.run();
//     std::cout << "program exit" << std::endl;
//     return 0;
// }