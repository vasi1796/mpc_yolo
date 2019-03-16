#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <atomic>

using boost::asio::ip::tcp;
using boost::asio::io_service;

class TCPServer {

private:
	io_service svc;
	tcp::acceptor acc;

	std::array<char, 32> buffer;
	std::thread serverThread;

	void handle_accept() {
		auto socket = std::make_shared<tcp::socket>(svc);
		acc.async_accept(*socket, [&, socket](boost::system::error_code error) {
			if (error)
				std::cerr << "client connection failed: " << error.message() << "\n";
			else {
				std::cout << "client connected\n";
				handle_session(socket);
			}
		});
	};

	void handle_session(std::shared_ptr<tcp::socket> socket) {
		socket->async_read_some(boost::asio::buffer(buffer), [&, socket](boost::system::error_code error, size_t bytes) {
			if (error)
				std::cout << "client disconnected: " << error.message() << "\n";

			else {

				if (strcmp(buffer.data(), "stop") == 0)
				{
					stop = true;
				}
				else if(strcmp(buffer.data(), "park") == 0)
				{
					park = true;
				}
				else
				{
					stop = false;
				}


				std::fill(std::begin(buffer), std::end(buffer), 0);

				handle_session(socket);
				handle_accept();
			}
		});
	};


public:

	std::atomic_bool stop;
	std::atomic_bool park;

	TCPServer() :
			acc(svc),
			stop(false),
			park(false)
	{
		acc.open(tcp::v4());
		acc.bind({ {},8888 });
		acc.listen(1);
	}

	~TCPServer() {
		if(serverThread.joinable())
			serverThread.join();
	}

	void run() {
		serverThread = std::thread([this] {
			handle_accept();
			svc.run();
		});
	}
};