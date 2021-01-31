#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// This is just a simple expansion of basic tutorial node
// that I use for experimenting with ROS concepts.
class SimpleTalker: public rclcpp::Node{
public:
	SimpleTalker(const char* node_name, const char* topic): Node(node_name){
		this->count = 0;
		this->publisher = this->create_publisher<std_msgs::msg::String>(topic,10);
		this->timer = this->create_wall_timer(500ms,
				std::bind(&SimpleTalker::timer_callback, this));
	}

private:
	rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    size_t count;

	void timer_callback(){
		auto msg = std_msgs::msg::String();
		msg.data = "Simlple talker revolution no. " + std::to_string(count++);
		RCLCPP_INFO(this->get_logger(), "Revolution no.: '%s'", msg.data.c_str());
		this->publisher->publish(msg);
	}
};

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
	auto talker = std::make_shared<SimpleTalker>("talker", "simple_msg");
	rclcpp::spin(talker);
    rclcpp::shutdown();
	return 0;
}
