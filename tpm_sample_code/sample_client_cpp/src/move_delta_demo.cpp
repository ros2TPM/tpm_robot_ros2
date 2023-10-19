#include "rclcpp/rclcpp.hpp"
#include "tpm_msgs/srv/robot_move.hpp"
#include "tpm_msgs/msg/robot_status.hpp"

#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

class SampleClient
{
  public:
    bool isBufferFull;
    bool isMoveDone;
    tpm_msgs::msg::RobotStatus robotStatus;
    std::array<float, 6> down = {0,0,-30,0,0,0};
    std::array<float, 6> up = {0,0,30,0,0,0};

    SampleClient(rclcpp::Node::SharedPtr node)
    {
      this->node = node;
      this->isBufferFull = 0;

      statusSubscriber = node->create_subscription<tpm_msgs::msg::RobotStatus>(
      "robot_status", 10, std::bind(&SampleClient::UpdateRobotStatus, this, std::placeholders::_1));

      client = node->create_client<tpm_msgs::srv::RobotMove>("/rob/move");
    }
    

    int move_ABS(std::array<float, 6> point){
    
      if (!client->wait_for_service(1s))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "'/rob/move' service not available..");
        return -1;
      }

      auto request = std::make_shared<tpm_msgs::srv::RobotMove::Request>();
      request->function = tpm_msgs::srv::RobotMove_Request::LIN_POSE;
      request->cmd_id = 0;
      request->mask = 0xff;
      
      request->pose = {0,0,0,0,0,0}; // move zrequest->pose = {0,0,-50,0,0,0}; // move z
      for(size_t i=0; i<6; i++){
        request->pose[i] = point[i];
        //std::cout <<"request "<<i<<"" <<request->pose[i] <<std::endl;
      }
      
      request->mp_data.feed = 500;
      request->mp_data.accel = 5000;
      request->mp_data.decel = 5000;
      request->mp_data.coord = tpm_msgs::msg::MotionProfile::COORD_ABS;

      auto result = client->async_send_request(request);
      rclcpp::spin_until_future_complete(node, result, 1s);
      auto res = result.get();
      if ((res->result_code) != 0)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute MovePTP. %s", res->message.c_str());
      return res->result_code;
    }


    int move_REL(std::array<float, 6> point){
    
      if (!client->wait_for_service(1s))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "'/rob/move' service not available..");
        return -1;
      }

      auto request = std::make_shared<tpm_msgs::srv::RobotMove::Request>();
      request->function = tpm_msgs::srv::RobotMove_Request::LIN_POSE;
      request->cmd_id = 0;
      request->mask = 0xff;
      
      request->pose = {0,0,0,0,0,0}; // move zrequest->pose = {0,0,-50,0,0,0}; // move z
      for(size_t i=0; i<6; i++){
        request->pose[i] = point[i];
        //std::cout <<"request "<<i<<"" <<request->pose[i] <<std::endl;
      }
      
      request->mp_data.feed = 500;
      request->mp_data.accel = 5000;
      request->mp_data.decel = 5000;
      request->mp_data.coord = tpm_msgs::msg::MotionProfile::COORD_REL;

      auto result = client->async_send_request(request);
      rclcpp::spin_until_future_complete(node, result, 1s);
      auto res = result.get();
      if ((res->result_code) != 0)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute MovePTP. %s", res->message.c_str());
      return res->result_code;
    }
    
    void sendCmd(std::array<float, 6> point){
      while(isBufferFull){
          rclcpp::sleep_for(std::chrono::milliseconds(1000));
          rclcpp::spin_some(node);
          printf("sleep because buffer full.\n");    
      }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node);

        int result;
        //printf( "move_ABS\n");
        result = move_ABS(point);
        if(result != 0)
            printf( "move_ABS fail. result=%d\n", result);

        //printf( "move_REL down\n");
        result = move_REL(down);
        if(result != 0)
            printf( "move_REL down fail. result=%d\n", result);

        //printf( "move_REL up\n");
        result = move_REL(up);
        if(result != 0)
            printf( "move_REL up fail. result=%d\n", result);   
    
    }

  private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<tpm_msgs::srv::RobotMove>::SharedPtr client;
    rclcpp::Subscription<tpm_msgs::msg::RobotStatus>::SharedPtr statusSubscriber;

    void UpdateRobotStatus(const tpm_msgs::msg::RobotStatus& robStatus)
    {
      robotStatus = robStatus;
      isBufferFull = (robotStatus.buffer_depth > 50);
      isMoveDone = (robotStatus.buffer_depth == 0);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "buff:%d, isbufferFull=%d", 
          robotStatus.buffer_depth,isBufferFull);

      std::string axisStr = "Axis:";
      std::string poseStr = "Pose:";
      for (size_t i = 0; i < robotStatus.axes.size(); i++)
        axisStr += std::to_string(robotStatus.axes[i].deg) + " ";

      for (size_t i = 0; i < robotStatus.axes.size(); i++)
        poseStr += std::to_string(robotStatus.end_pose[i]) + " ";

      
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", axisStr.c_str());
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s\n", poseStr.c_str());
    }

    
    
};

void sendCmd(SampleClient client, std::shared_ptr<rclcpp::Node> node, std::array<float, 6> point){
    while(client.isBufferFull){
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        rclcpp::spin_some(node);
        printf("sleep because buffer full.\n");    
    }

      int result;
      //printf( "move_ABS\n");
      result = client.move_ABS(point);
      if(result != 0)
          printf( "move_ABS fail. result=%d\n", result);

      //printf( "move_REL down\n");
      //result = client.move_REL(client.down);
      //if(result != 0)
      //    printf( "move_REL down fail. result=%d\n", result);

      //printf( "move_REL up\n");
      //result = client.move_REL(client.up);
      //if(result != 0)
      //    printf( "move_REL up fail. result=%d\n", result);   
  
}

int main(int argc, char ** argv)
{
  std::vector<std::array<float, 6>> points{{140,0,-570,0,0,0},                                   
                                           {100,-100,-570,0,0,0},
                                           {0,-140,-570,0,0,0},
                                           {-100,-100,-570,0,0,0},
                                           {-140,0,-570,0,0,0},
                                           {-100,100,-570,0,0,0},
                                           {0,140,-570,0,0,0},
                                           {100,100,-570,0,0,0}
                                           };
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sample_client");
  SampleClient client(node);
  char c = 0;
  int round = 0;
  //while (true)
		{
			std::cout << "\n-Please type command (h for help): \n";

			std::cin >> c;
			switch (c)
			{
  
      case 'r':
        while(true) {
          round++;
          for(size_t i=0; i<points.size(); i++){
            printf("send cmd %d - %u\n",round, i);;
            client.sendCmd( points[i]);
            
          }
          
        }
        break;

      case 't':
          for(size_t i=0; i<points.size(); i++){
            printf("send cmd %u\n", i);;
            sendCmd(client, node, points[i]);
          }
          
        break;  

      case 'q':
        rclcpp::shutdown();
        return 0;
			}
    }
}