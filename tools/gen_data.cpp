#include <iostream>
#include <fstream>
#include <numeric>
#include <vector>
#include <random>

using point = std::pair<double, double>;

std::vector<point> landmarks = {
  {-1.0,-1.0}, { 1.0,-1.0}, {-1.0, 1.0}, { 1.0, 1.0}
};

struct Simulator
{

  double
  add_noise(double value)
  {
    return value + observation_distribution(generator);
  }

  point
  compute_observation(const point & robot_position, const point & landmark)
  {
    return { add_noise(landmark.first - robot_position.first), add_noise(landmark.second - robot_position.second) };
  }

  point
  random_robot_location()
  {
    return {robot_location_distribution(generator), robot_location_distribution(generator)};
  }

  std::normal_distribution<double> observation_distribution{0.0, 0.1};
  std::uniform_real_distribution<> robot_location_distribution{-1.0, 1.0};
  std::default_random_engine generator;
};

int
main(int argc, char *  argv[])
{
  const std::size_t observations_count = 10000;
  std::ofstream myfile("observations.txt");

  myfile << landmarks.size() << " " << observations_count << "\n";

  Simulator sim;

  for(std::size_t i = 0; i < observations_count; ++i)
  {
    point robot_pos = i == 0 ? point(0, 0) : sim.random_robot_location();
    for(const point& landmark : landmarks)
    {
      point obs = sim.compute_observation(robot_pos, landmark);
      myfile << obs.first << " " << obs.second << " ";
    }
    myfile << "\n";
  }

  myfile.close();
  return 0;
}
