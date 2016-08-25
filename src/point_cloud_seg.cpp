// Point cloud rendering utilities
#include <render_utils/render_utils.h>

// File reading utils
#include <file_utils/file_utils.h>

// Standard Libraries
#include <iostream>

int main(int argc, const char * argv[]) 
{
	std::string message = "Hello world";
	auto message_split = file_utils::split_string_spaces(message);

	for(auto word : message_split)
	{
		std::cout << word << std::endl;
	}
	
    return 0;
}
