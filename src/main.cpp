#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

using point = std::pair<double, double>;

int
main(int argc, char *  argv[])
{

  std::ifstream myfile("observations.txt");
	std::size_t landmarks_count, observations_count;
	myfile >> landmarks_count >> observations_count;

	std::vector<point> current_map;

	// First position is used to initialize the map
	for(std::size_t i = 0; i < landmarks_count; ++i)
	{
		point pt;
		myfile >> pt.first >> pt.second;
		current_map.push_back(pt);
	}

	// Then process the observation
	for(std::size_t iter = 1; iter < observations_count; ++iter)
	{
		// First estimate the robot position
		point estimated_robot_pos{0.0, 0.0};
		std::vector<point> observations;
		for(std::size_t i = 0; i < landmarks_count; ++i)
		{
			point pt;
			myfile >> pt.first >> pt.second;
			observations.push_back(pt);

			estimated_robot_pos.first += current_map[i].first - pt.first;
			estimated_robot_pos.second += current_map[i].second - pt.second;
		}
		estimated_robot_pos.first /= landmarks_count;
		estimated_robot_pos.second /= landmarks_count;

		// Update map
		for(std::size_t i = 0; i < landmarks_count; ++i)
		{
			point& map_point = current_map[i];
			point obs_point = observations[i];
			map_point.first = (iter * map_point.first + (obs_point.first + estimated_robot_pos.first)) / (iter + 1);
			map_point.second = (iter * map_point.second + (obs_point.second + estimated_robot_pos.second)) / (iter + 1);
		}
		if(iter % 100 == 0)
		{
			for(std::size_t i = 0; i < landmarks_count; ++i)
			{
				std::cout << "Iter: " << iter << " landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << std::endl;
			}

		}
	}

	for(std::size_t i = 0; i < landmarks_count; ++i)
	{
		std::cout << "Final iter landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << std::endl;
	}
	return 0;
}
