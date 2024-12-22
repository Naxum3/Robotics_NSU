#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CircularMotionRobot : public rclcpp::Node
{
public:
    CircularMotionRobot()
        : Node("circular_motion_robot"),
          safe_distance_(1.0), // Безопасное расстояние до препятствия (1 метр)
          is_obstacle_(false)
    {
        // Создаем издателя для отправки команд управления
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        // Создаем таймер для регулярного вызова функции управления
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CircularMotionRobot::control_callback, this));

        // Подписываемся на топик с изображением глубины
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image", 10,
            std::bind(&CircularMotionRobot::depth_image_callback, this, std::placeholders::_1));
    }

private:
    void control_callback()
    {
        auto msg = geometry_msgs::msg::Twist();

        if (is_obstacle_)
        {
            // Останавливаем робота, если препятствие обнаружено
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stopping robot.");
        }
        else
        {
            // Двигаемся по кругу
            msg.linear.x = 1;  // Линейная скорость вперед
            msg.angular.z = 0.5; // Угловая скорость для движения по кругу
            RCLCPP_INFO(this->get_logger(), "No obstacles. Moving in a circle.");
        }

        publisher_->publish(msg);
    }

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Преобразуем ROS-сообщение в OpenCV-изображение
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Получаем изображение глубины
        cv::Mat depth_image = cv_ptr->image;

        // Определяем центральную точку изображения
        int center_x = depth_image.cols / 2;
        int center_y = depth_image.rows / 2;

        // Получаем значение глубины в центральной точке
        float center_depth = depth_image.at<float>(center_y, center_x);

        // Проверяем, находится ли препятствие в пределах безопасного расстояния
        is_obstacle_ = (std::isfinite(center_depth) && center_depth > 0 && center_depth < safe_distance_);

        RCLCPP_INFO(this->get_logger(), "Depth at center: %f", center_depth);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    float safe_distance_; // Безопасное расстояние до препятствия
    bool is_obstacle_;    // Флаг наличия препятствия
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularMotionRobot>());
    rclcpp::shutdown();
    return 0;
}
