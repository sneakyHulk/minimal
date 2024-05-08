#include <ecal/ecal.h>
#include <ecal/msg/messagepack/publisher.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <random>
#include <thread>

#include "common_output.h"
#include "timestamp.h"

namespace {
	volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int signal) {
	common::println("Got SIGINT/SIGTERM! Will finalize eCAL.");
	gSignalStatus = signal;
}

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> distrib(1, 6);

int main(int argc, char **argv) {
	eCAL::Initialize(argc, argv, "pub");
	std::signal(SIGTERM, signal_handler);
	std::signal(SIGINT, signal_handler);

	eCAL::messagepack::CPublisher<Timestamp> timestamp_publisher("timestamp");

	while (!gSignalStatus && eCAL::Ok()) {
		auto const now = std::chrono::system_clock::now();
		auto const now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

		Timestamp timestamp;
		timestamp.ns = now_ns.time_since_epoch().count();

		for (auto i = 0; i < distrib(gen); ++i) timestamp.data.push_back(distrib(gen));

		timestamp_publisher.Send(timestamp);

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	eCAL::Finalize();

	return 0;
}

/*
#include <ecal/msg/string/publisher.h>

int main(int argc, char **argv) {
    // Initialize eCAL. The name of our Process will be "Hello World Publisher"
    eCAL::Initialize(argc, argv, "Hello World Publisher");

    // Create a String Publisher that publishes on the topic "hello_world_topic"
    eCAL::string::CPublisher<std::string> publisher("hello_world_topic");

    // Create a counter, so something changes in our message
    int counter = 0;

    // Infinite loop (using eCAL::Ok() will enable us to gracefully shutdown the
    // Process from another application)
    while (eCAL::Ok()) {
        // Create a message with a counter an publish it to the topic
        std::string message = "Hello World " + std::to_string(++counter);
        std::cout << "Sending message: " << message << std::endl;

        publisher.Send(message);

        // Sleep 500 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // finalize eCAL API
    eCAL::Finalize();

    return 0;
}
*/

/*
#include <capnp/dynamic.h>
#include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/publisher.h>

#include "timestamp.capnp.h"

int main(int argc, char **argv) {
    eCAL::Initialize(argc, argv, "TESTING");
    eCAL::capnproto::CPublisher<Timestamp> timestamp_publisher("time");

    while (eCAL::Ok()) {
        auto now = std::chrono::system_clock::now();
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

        auto timestamp = timestamp_publisher.GetBuilder();
        timestamp.setNs(now_ns.time_since_epoch().count());
        timestamp_publisher.Send();

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    eCAL::Finalize();

    return 0;
}
*/