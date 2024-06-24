#pragma once

#include <oneapi/tbb/concurrent_queue.h>

#include <memory>
#include <ranges>
#include <utility>

template <typename Output>
class InputNode;

template <typename Input>
class OutputNode {
   protected:
	tbb::concurrent_queue<std::shared_ptr<Input>> _input_queue;

	template <typename T>
	friend class InputNode;

	void input(std::shared_ptr<Input> input) { _input_queue.push(std::forward<decltype(input)>(input)); }

   public:
	[[noreturn]] virtual void operator()() {
		std::shared_ptr<Input> item;
		while (true) {
			if (_input_queue.try_pop(item)) {
				output_function(*item);
			} else {
				std::this_thread::yield();
			}
		}
	}
	virtual void output_function(Input const&) = 0;
};

template <typename Output>
class InputNode {
   protected:
	std::vector<std::function<void(std::shared_ptr<Output>)>> _output_connections;

   public:
	[[noreturn]] virtual void operator()() {
		while (true) {
			std::shared_ptr<Output> output = std::make_shared<Output>(input_function());

			for (auto const& connection : _output_connections) connection(output);
		}
	}

	virtual Output input_function() = 0;

	void operator+=(OutputNode<Output>& node) { _output_connections.push_back(std::bind(&OutputNode<Output>::input, &node, std::placeholders::_1)); }
};

template <typename Input, typename Output>
class InputOutputNode : public InputNode<Output>, public OutputNode<Input> {
	Output input_function() final { std::unreachable(); };
	void output_function(Input const&) final {};

   public:
	[[noreturn]] virtual void operator()() {
		while (true) {
			std::shared_ptr<Input> item;
			if (OutputNode<Input>::_input_queue.try_pop(item)) {
				std::shared_ptr<Output> output = std::make_shared<Output>(function(*item));

				for (auto const& connection : InputNode<Output>::_output_connections) connection(output);
			} else {
				std::this_thread::yield();
			}
		}
	}
	virtual Output function(Input const&) = 0;
};