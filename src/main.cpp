#include <fstream>
#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

#if 1

#include <uxhw.h>

#else

// approximate signaloid API

double UxHwDoubleGaussDist(double mu, double /*sigma*/)
{
	return mu;
}

double  UxHwDoubleUniformDist(double a, double b)
{
	return 0.5 * (a + b);
}

double  UxHwDoubleBayesLaplace(double (*likelihood)(double), double prior, double evidence)
{
	return 0.5 * (prior + evidence);
}

#endif


#include <random>

std::default_random_engine generator;

double landmark_sensor_uncertainty = 0.03;
double robot_position_sensor_uncertainty = 0.05;

double noisy_landmark_sensor(double measurand)
{
	return UxHwDoubleGaussDist(measurand, landmark_sensor_uncertainty);
}

double normal_distribution(double mu, double sigma)
{
  std::normal_distribution<double> gauss_dist{mu, sigma};
	return UxHwDoubleGaussDist(gauss_dist(generator), sigma);
}

double  uniform_distribution(double a, double b)
{
	std::uniform_real_distribution<> uniform_dist{a, b};
	double v = uniform_dist(generator);
	return UxHwDoubleUniformDist(v + a, v + b);
}


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
compute_landmark_observation(const point & robot_position, const point & landmark, double uncertainty = landmark_sensor_uncertainty)
{
	return { normal_distribution(landmark.first - robot_position.first, landmark_sensor_uncertainty), normal_distribution(landmark.second - robot_position.second, landmark_sensor_uncertainty) };
}

point
compute_robot_position_observation(const point & robot_position)
{
	return { normal_distribution(robot_position.first, robot_position_sensor_uncertainty), normal_distribution(robot_position.second, robot_position_sensor_uncertainty) };
}

point
random_robot_location()
{
	return {uniform_distribution(-1.0, 1.0), uniform_distribution(-1.0, 1.0)};
}

double
compute_error(const point& _a, const point& _b)
{
	return std::sqrt(std::pow(_a.first - _b.first, 2) + std::pow(_a.second - _b.second, 2));
}

int
main(int argc, char *  argv[])
{
	std::size_t observations_count = 100;

	std::vector<point> current_map;

	// First position is used to initialize the map
	for(std::size_t i = 0; i < landmarks.size(); ++i)
	{
		point pt = compute_landmark_observation({0, 0}, landmarks[i],
											landmark_sensor_uncertainty + robot_position_sensor_uncertainty);
		current_map.push_back(pt);
	}
	std::cout << "========== Initial map ==========" << std::endl;
	for(std::size_t i = 0; i < current_map.size(); ++i)
	{
		std::cout << "Landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << " err: " << compute_error(current_map[i], landmarks[i]) << std::endl;
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
		point robot_pos_observation = compute_robot_position_observation(robot_pos);

		// First measure the robot position

		// Update map
		for(std::size_t i = 0; i < observations.size(); ++i)
		{
			point& map_point = current_map[i];
			point obs_point = observations[i];
			map_point.first = UxHwDoubleBayesLaplace(&noisy_landmark_sensor, map_point.first, robot_pos_observation.first + obs_point.first);
			map_point.second = UxHwDoubleBayesLaplace(&noisy_landmark_sensor, map_point.second, robot_pos_observation.second + obs_point.second);
			std::cout << robot_pos_observation.first + obs_point.first << " " << robot_pos_observation.second + obs_point.second << std::endl;
		}
		// if(iter % 100 == 1)
		{
			std::cout << "=========== Iter " << iter << "===========" << std::endl;
			std::cout << "Actual robot position: x: " << robot_pos.first << " y: " << robot_pos.second << std::endl;
			std::cout << "Observed robot position: x: " << robot_pos_observation.first << " y: " << robot_pos_observation.second << " err: " << compute_error(robot_pos, robot_pos_observation) << std::endl;
			double total_err = 0.0;
			for(std::size_t i = 0; i < current_map.size(); ++i)
			{
				double err = compute_error(current_map[i], landmarks[i]);
				std::cout << "Landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << " err: " << err << std::endl;
				total_err += err;
			}
			std::cout << "Total err: " << total_err << std::endl;
			for(std::size_t i = 0; i < observations.size(); ++i)
			{
				std::cout << "Observation: " << i << " x: " << observations[i].first << " y: " << observations[i].second << std::endl;
			}

		}
	}

	std::cout << "========== Ground truth ==========" << std::endl;
	for(std::size_t i = 0; i < current_map.size(); ++i)
	{
		std::cout << "Landmark: " << i << " x: " << landmarks[i].first << " y: " << landmarks[i].second << std::endl;
	}
	std::cout << "=========== Estimated ============" << std::endl;
	for(std::size_t i = 0; i < current_map.size(); ++i)
	{
		std::cout << "Landmark: " << i << " x: " << current_map[i].first << " y: " << current_map[i].second << " err: " << compute_error(current_map[i], landmarks[i]) << std::endl;
	}
	return 0;
}
