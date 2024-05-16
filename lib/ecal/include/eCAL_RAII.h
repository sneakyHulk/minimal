#pragma once

#include <ecal/ecal.h>

#include <filesystem>
#include <source_location>

#include "common_output.h"

class eCAL_RAII {
   public:
#ifdef __cpp_lib_source_location
	eCAL_RAII(int const& argc, char** const& argv, std::filesystem::path const& name = std::filesystem::path(std::source_location::current().file_name()).stem()){
#else
	eCAL_RAII(int const& argc, char** const& argv, std::filesystem::path const& name) {
#endif
	    eCAL::Initialize(argc, argv, name.c_str());
		common::println("Initialized eCAL unit with unit_name '", name.c_str(), "'!");
	}

~eCAL_RAII() {
		eCAL::Finalize();
		common::println("Finalized eCAL unit!");
	}
};
