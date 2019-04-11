#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <atomic>

using boost::asio::ip::tcp;
using boost::asio::io_service;
using boost::system::error_code;

class Client {
private:
	io_service ios;
	tcp::socket sock;
	error_code error;

	std::array<char, 32> buffer;

public:

	Client(std::string ip) :sock(ios) 
    {
		tcp::endpoint endpoint = tcp::endpoint(boost::asio::ip::address::from_string(ip), 8888);
		sock.connect(endpoint, error);

		if (error)
			std::cout << "can't connect to server: " << error.message() << "\n";
		else
			std::cout << "connected to server\n";
	}

	void send(std::string message) 
    {
		std::copy(message.begin(), message.end(), buffer.begin());
		sock.write_some(boost::asio::buffer(buffer, message.size()), error);
		if (error)
			std::cout << "server closed: " << error.message() << "\n";
	}
};
