#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "worhole_nav/action/multi_map_nav.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class WormholeNavigator : public rclcpp::Node {
public:
    using MultiMapNav = worhole_nav::action::MultiMapNav;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MultiMapNav>;

    WormholeNavigator() : Node("wormhole_navigator") {
        // ✅ FIX: Use absolute topic name
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        action_server_ = rclcpp_action::create_server<MultiMapNav>(
            this,
            "navigate_multimap",
            std::bind(&WormholeNavigator::handle_goal, this, _1, _2),
            std::bind(&WormholeNavigator::handle_cancel, this, _1),
            std::bind(&WormholeNavigator::handle_accepted, this, _1)
        );

        if (sqlite3_open("install/worhole_nav/share/worhole_nav/config/wormholes.db", &db_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open wormhole database.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to wormhole database.");
        }

        current_map_ = "room1"; // You can dynamically update this
    }

private:
    rclcpp_action::Server<MultiMapNav>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    sqlite3* db_;
    std::string current_map_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MultiMapNav::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{std::bind(&WormholeNavigator::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f) in map [%s]", goal->x, goal->y, goal->map_name.c_str());

        if (goal->map_name == current_map_) {
            send_goal_to_nav(goal->x, goal->y);
        } else {
            double wx, wy;
            if (get_wormhole_to_map(goal->map_name, wx, wy)) {
                send_goal_to_nav(wx, wy);
                switch_map(goal->map_name);
                send_goal_to_nav(goal->x, goal->y);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Wormhole to [%s] not found.", goal->map_name.c_str());
            }
        }

        auto result = std::make_shared<MultiMapNav::Result>();
        result->success = true;
        result->message = "Navigation completed.";
        goal_handle->succeed(result);
    }

    bool get_wormhole_to_map(const std::string& to_map, double &x, double &y) {
        std::string query = "SELECT to_x, to_y FROM wormholes WHERE from_map='" + current_map_ + "' AND to_map='" + to_map + "'";
        sqlite3_stmt* stmt;
        bool found = false;

        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
            if (sqlite3_step(stmt) == SQLITE_ROW) {
                x = sqlite3_column_double(stmt, 0);
                y = sqlite3_column_double(stmt, 1);
                found = true;
            }
            sqlite3_finalize(stmt);
        }
        return found;
    }

    void send_goal_to_nav(double x, double y) {
        while (!nav_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for navigate_to_pose server...");
        }

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto goal_future = nav_client_->async_send_goal(goal_msg);
        auto goal_handle = goal_future.get();

        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the navigate_to_pose action server.");
            return;
        }

        auto result_future = nav_client_->async_get_result(goal_handle);
        auto result = result_future.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach goal.");
        }
    }

    void switch_map(const std::string& new_map) {
        std::string map_path = "install/worhole_nav/share/worhole_nav/maps/" + new_map + ".yaml";
        std::string cmd = "./install/worhole_nav/lib/worhole_nav/switch_map.sh " + map_path;
        RCLCPP_INFO(this->get_logger(), "Switching map to %s", map_path.c_str());
        system(cmd.c_str());
        current_map_ = new_map;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WormholeNavigator>());
    rclcpp::shutdown();
    return 0;
}
