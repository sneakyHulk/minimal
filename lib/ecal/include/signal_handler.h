#pragma once

#include <csignal>

#include "common_output.h"

class signal_handler {
   public:
	static volatile std::sig_atomic_t gSignalStatus;
	signal_handler();
};

volatile std::sig_atomic_t signal_handler::gSignalStatus = 0;

void handler(int signal) {
	common::println("Got SIGINT/SIGTERM!");
	signal_handler::gSignalStatus = signal;
}

signal_handler::signal_handler() {
	std::signal(SIGTERM, handler);
	std::signal(SIGINT, handler);
}
