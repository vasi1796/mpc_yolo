#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <thread>

using boost::asio::ip::tcp;
using boost::asio::io_service;
using boost::system::error_code;

class Server {

private:
	io_service svc;
	tcp::acceptor acc;

	std::array<char, 32> buffer;
	std::thread serverThread;


	void handle_accept() {
		auto socket = std::make_shared<tcp::socket>(svc);
		acc.async_accept(*socket, [&, socket](error_code error) {
			if (error)
				std::cerr << "client connection failed: " << error.message() << "\n";
			else {
				printf("client connected\n");
				handle_session(socket);
			}
		});
	};

	void handle_session(std::shared_ptr<tcp::socket> socket) {
		socket->async_read_some(boost::asio::buffer(buffer), [&, socket](error_code error, size_t bytes) {
			if (error)
				std::cout << "client disconnected: " << error.message() << "\n";

			else {
				printf(buffer.data());
				std::fill(std::begin(buffer), std::end(buffer), 0);

				handle_session(socket);
				handle_accept();
			}
		});
	};


public:

	Server() :acc(svc) {
		acc.open(tcp::v4());
		acc.bind({ {},8888 });
		acc.listen(1);
	}

	void run() {
		serverThread = std::thread([this] {
			handle_accept();
			svc.run();
		});
	}
};
