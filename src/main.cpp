#include <fstream>
#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

#if 1

#include <uxhw.h>

#else

#include <random>

// emulate signaloid API

std::default_random_engine generator;

double UxHwDoubleGaussDist(double mu, double sigma)
{
  std::normal_distribution<double> gauss_dist{mu, sigma};
	return gauss_dist(generator);
}

double  UxHwDoubleUniformDist(double a, double b)
{
	std::uniform_real_distribution<> uniform_dist{-1.0, 1.0};
	return uniform_dist(generator);
}

#endif

using point = std::pair<double, double>;

std::ostream&
operator<<(std::ostream& __os,
			const point& __pt)
{
	__os << "(" << __pt.first << ", " << __pt.second << ")";
	return __os;
}

std::vector<point> landmarks = {
  {-1.0,-1.0}, { 1.0,-1.0}, {-1.0, 1.0}, { 1.0, 1.0}
};

point
compute_landmark_observation(const point & robot_position, const point & landmark)
{
	return { UxHwDoubleGaussDist(landmark.first - robot_position.first, 0.1), UxHwDoubleGaussDist(landmark.second - robot_position.second, 0.1) };
}

point
compute_robot_position_observation(const point & robot_position)
{
	return { UxHwDoubleGaussDist(robot_position.first, 0.1), UxHwDoubleGaussDist(robot_position.second, 0.1) };
}

point
random_robot_location()
{
	return {UxHwDoubleUniformDist(-1.0, 1.0), UxHwDoubleUniformDist(-1.0, 1.0)};
}

double
compute_error(const point& _a, const point& _b)
{
	return std::sqrt(std::pow(_a.first - _b.first, 2) + std::pow(_a.second - _b.second, 2));
}

int
main(int argc, char *  argv[])
{
	std::size_t observations_count = 10000;

	std::vector<point> current_map;

	// First position is used to initialize the map
	for(std::size_t i = 0; i < landmarks.size(); ++i)
	{
		// point pt = compute_landmark_observation({0, 0}, landmarks[i]);
		point pt = landmarks[i];
		current_map.push_back(pt);
	}

	// Then process the observation
	for(std::size_t iter = 1; iter < observations_count; ++iter)
	{
		// Simulate observations
		point robot_pos = random_robot_location();
		std::vector<point> observations;
		for(std::size_t i = 0; i < landmarks.size(); ++i)
		{
			observations.push_back(compute_landmark_observation(robot_pos, landmarks[i]));
		}

		// First estimate the robot position
		point estimated_robot_pos = {0, 0};
		for(std::size_t i = 0; i < observations.size(); ++i)
		{
			estimated_robot_pos.first += current_map[i].first - observations[i].first;
			estimated_robot_pos.second += current_map[i].second - observations[i].second;
		}
		estimated_robot_pos.first /= observations.size();
		estimated_robot_pos.second /= observations.size();

		// Update map
		for(std::size_t i = 0; i < observations.size(); ++i)
		{
			point& map_point = current_map[i];
			point obs_point = observations[i];
			map_point.first = (iter * map_point.first + (obs_point.first + estimated_robot_pos.first)) / (iter + 1);
			map_point.second = (iter * map_point.second + (obs_point.second + estimated_robot_pos.second)) / (iter + 1);
		}
		if(iter % 100 == 0)
		{
			std::cout << "Position: " << compute_error(robot_pos, estimated_robot_pos) << std::endl;
			for(std::size_t i = 0; i < observations.size(); ++i)
			{
				std::cout << "Iter: " << iter << " landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << " err: " << compute_error(current_map[i], landmarks[i]) << std::endl;
			}

		}
	}

	for(std::size_t i = 0; i < current_map.size(); ++i)
	{
		std::cout << "Final iter landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << std::endl;
	}
	return 0;
}
