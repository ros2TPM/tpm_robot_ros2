#include <rclcpp/rclcpp.hpp> //for logger

#define ROS_PRINT(...) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__);


#define TRY_AND_PRINT(func) do { \
	auto ret = (func);\
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"(%d) in %s", ret, #func);\
	if(ret != 0) {\
		return ret; \
		} \
	}while(0)

#define TRY(func) do { \
	auto ret = (func);\
	if(ret != 0) {\
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"(%d) in %s", ret, #func);\
		return ret; \
		} \
	}while(0) 
