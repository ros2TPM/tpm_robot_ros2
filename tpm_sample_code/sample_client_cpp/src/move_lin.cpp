#include "rclcpp/rclcpp.hpp"
#include "tpm_msgs/srv/robot_move.hpp"
#include "tpm_msgs/msg/robot_status.hpp"

#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

class SampleClient
{
  public:
    bool isMoveDone;
    tpm_msgs::msg::RobotStatus robotStatus;

    SampleClient(rclcpp::Node::SharedPtr node)
    {
      this->node = node;

      statusSubscriber = node->create_subscription<tpm_msgs::msg::RobotStatus>(
      "robot_status", 10, std::bind(&SampleClient::UpdateRobotStatus, this, std::placeholders::_1));

      client = node->create_client<tpm_msgs::srv::RobotMove>("/rob/move");
    }

    int move_lin()
    {
      if (!client->wait_for_service(1s))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "'/rob/move' service not available..");
        return -1;
      }
      isMoveDone = false;

      auto request = std::make_shared<tpm_msgs::srv::RobotMove::Request>();
      request->function = tpm_msgs::srv::RobotMove_Request::LIN_POSE;
      request->cmd_id = 0;
      request->mask = 0xff;
      request->pose = {0,0,-50,0,0,0}; // move z
      request->mp_data.feed = 40;
      request->mp_data.accel = 100;
      request->mp_data.decel = 100;
      request->mp_data.coord = tpm_msgs::msg::MotionProfile::COORD_REL;

      auto result = client->async_send_request(request);
      rclcpp::spin_until_future_complete(node, result, 1s);

      auto res = result.get();
      if ((res->result_code) != 0)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute MovePTP. %s", res->message.c_str());

      return res->result_code;
    }
    
  private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<tpm_msgs::srv::RobotMove>::SharedPtr client;
    rclcpp::Subscription<tpm_msgs::msg::RobotStatus>::SharedPtr statusSubscriber;

    void UpdateRobotStatus(const tpm_msgs::msg::RobotStatus& robStatus)
    {
      robotStatus = robStatus;
      isMoveDone = (robotStatus.buffer_depth == 0);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "buff:%d", robotStatus.buffer_depth);

      std::string axisStr = "Axis:";
      std::string poseStr = "Pose:";
      for (size_t i = 0; i < robotStatus.axes.size(); i++)
        axisStr += std::to_string(robotStatus.axes[i].deg) + " ";

      for (size_t i = 0; i < robotStatus.axes.size(); i++)
        poseStr += std::to_string(robotStatus.end_pose[i]) + " ";

      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", axisStr.c_str());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s\n", poseStr.c_str());
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sample_client");
  SampleClient client(node);

  int result = client.move_lin();
  if (result == 0)
  {
    do {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node);
    } while (!client.isMoveDone);
  }

  rclcpp::shutdown();
  return 0;
}
