#include <ecal/ecal.h>
#include <ecal/msg/messagepack/subscriber.h>

#include <chrono>
#include <csignal>
#include <iostream>
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

void OnTimestamp(Timestamp const& msg) {
	std::chrono::nanoseconds const just_now_ns(msg.ns);
	std::chrono::time_point<std::chrono::system_clock> const just_now(just_now_ns);

	auto const now = std::chrono::system_clock::now();

	auto const diff = now - just_now;
	auto const diff_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);

	common::println(msg.data);

	std::cout << "Time taken = " << diff_ms.count() << " ns" << std::endl;
}

int main(int argc, char** argv) {
	eCAL::Initialize(argc, argv, "sub");
	std::signal(SIGTERM, signal_handler);
	std::signal(SIGINT, signal_handler);

	eCAL::messagepack::CSubscriber<Timestamp> timestamp_subscriber("timestamp");
	timestamp_subscriber.AddReceiveCallback(std::bind(OnTimestamp, std::placeholders::_2));

	while (!gSignalStatus && eCAL::Ok())
		;

	eCAL::Finalize();

	return 0;
}

/*
#include <ecal/msg/string/subscriber.h>


void HelloWorldCallback(const std::string& message) { std::cout << "Received Message: " << message << std::endl; }

int main2(int argc, char** argv) {
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "Hello World Subscriber");

    // Create a subscriber that listenes on the "hello_world_topic"
    eCAL::string::CSubscriber<std::string> subscriber("hello_world_topic");

    // Set the Callback
    subscriber.AddReceiveCallback(std::bind(&HelloWorldCallback, std::placeholders::_2));

    // Just don't exit
    while (eCAL::Ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // finalize eCAL API
    eCAL::Finalize();

    return 0;
}
*/

/*
#include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/subscriber.h>

#include "timestamp.capnp.h"

void OnTimestamp(Timestamp::Reader msg_) {
    std::chrono::nanoseconds just_now_ns(msg_.getNs());
    std::chrono::time_point<std::chrono::system_clock> just_now(just_now_ns);

    auto now = std::chrono::system_clock::now();

    auto diff = now - just_now;
    auto diff_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);
    std::cout << "Time taken = " << diff_ms.count() << " ns" << std::endl;
}

int main(int argc, char** argv) {
    eCAL::Initialize(argc, argv, "TESTING");
    eCAL::capnproto::CSubscriber<Timestamp> timestamp_subscriber("time");

    timestamp_subscriber.AddReceiveCallback(std::bind(OnTimestamp, std::placeholders::_2));

    while (eCAL::Ok()) {
    }

    eCAL::Finalize();

    return 0;
}
*/