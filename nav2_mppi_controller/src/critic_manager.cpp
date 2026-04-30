// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  const size_t batch = data.costs.shape(0);
  std::vector<std::pair<std::string, xt::xtensor<float, 1>>> per_critic_costs;
  per_critic_costs.reserve(critics_.size());

  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }
    xt::xtensor<float, 1> costs_before = data.costs;
    critics_[q]->score(data);
    per_critic_costs.emplace_back(
      critics_[q]->getName(),
      data.costs - costs_before);
  }

  if (!per_critic_costs.empty() && batch > 0) {
    size_t best = 0;
    for (size_t i = 1; i < batch; ++i) {
      if (data.costs(i) < data.costs(best)) { best = i; }
    }
    float total = data.costs(best);
    std::string msg = "[CriticManager] Optimal traj#" + std::to_string(best) +
      " | total=" + std::to_string(total);
    for (auto & [name, costs] : per_critic_costs) {
      msg += "\n  " + name + ": " + std::to_string(costs(best));
    }
    RCLCPP_INFO(logger_, "%s", msg.c_str());

    for (auto & [name, costs] : per_critic_costs) {
      float min_cost = costs(0);
      float max_cost = costs(0);
      double sum_cost = 0.0;
      for (size_t i = 0; i < batch; ++i) {
        min_cost = std::min(min_cost, costs(i));
        max_cost = std::max(max_cost, costs(i));
        sum_cost += costs(i);
      }

      RCLCPP_INFO(
        logger_,
        "[CriticStats] %s: min=%.6f max=%.6f mean=%.6f best=%.6f",
        name.c_str(), static_cast<double>(min_cost), static_cast<double>(max_cost),
        sum_cost / static_cast<double>(batch), static_cast<double>(costs(best)));
    }
  }
}

void CriticManager::logObstacleCriticScores(
  CriticData & data, const std::string & label) const
{
  for (const auto & critic : critics_) {
    const std::string critic_name = critic->getName();
    if (critic_name.find("ObstaclesCritic") == std::string::npos) {
      continue;
    }

    xt::xtensor<float, 1> costs_before = data.costs;
    const bool fail_before = data.fail_flag;
    critic->score(data);
    const float cost = (data.costs - costs_before)(0);

    RCLCPP_INFO(
      logger_, "[CriticManager] %s %s: %.6f%s",
      label.c_str(), critic_name.c_str(), static_cast<double>(cost),
      data.fail_flag ? " | fail=true" : "");

    data.fail_flag = fail_before;
  }
}

}  // namespace mppi
