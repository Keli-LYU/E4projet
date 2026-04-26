// =====================================================================================
//
//       Filename:  TwistToRegulatedControl.cpp
//
//    Description:  Node to convert twist messages to carla control messages
//
//        Version:  1.0
//        Created:  11/10/2022 17:13:37
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Thibaud Dubautbout (TD), thibaud.duhautbout@expleogroup.com
//   Organization:  Expleo Group
//
// =====================================================================================
#include "carla_twist_to_regulated_control/TwistToRegulatedControl.hpp"

using std::placeholders::_1;

TwistToRegulatedControl::TwistToRegulatedControl()
    : rclcpp::Node("twist_to_regulated_control")
{
    // declare parameters
    declare_node_parameters();

    // initialize PID controller
    _speedController = PIDController(
        this->get_parameter("SpeedController.Kp").as_double(),
        this->get_parameter("SpeedController.Ki").as_double(),
        this->get_parameter("SpeedController.Kd").as_double(),
        this->get_parameter("SpeedController.int_sat_min").as_double(),
        this->get_parameter("SpeedController.int_sat_max").as_double(),
        this->get_parameter("SpeedController.sat_min").as_double(),
        this->get_parameter("SpeedController.sat_max").as_double());

    //-----------------------------------------------------------------------------
    // initialize topics
    //-----------------------------------------------------------------------------

    // twist subscriber
    _twistSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        this->get_parameter("twist_input_topic").as_string(),
        10,
        std::bind(&TwistToRegulatedControl::twistCallback, this, _1));

    // odom subscriber
    _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odometry_input_topic").as_string(),
        10,
        std::bind(&TwistToRegulatedControl::odomCallback, this, _1));

    _controlPublisher =
        this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            this->get_parameter("carla_control_topic").as_string(),
            10);

    _vehicleInfo.delta_max = 1.22;
    _vehicleInfo.L = 2.2;
    _vehicleInfo.initialized = true;
}

void TwistToRegulatedControl::declare_node_parameters()
{
    // input twist topic
    this->declare_parameter<std::string>("twist_input_topic", "/cmd_vel");

    // input odometry topic
    this->declare_parameter<std::string>("odometry_input_topic",
                                         "/carla/ego_vehicle/odometry");

    // output command topic
    this->declare_parameter<std::string>(
        "carla_control_topic",
        "/carla/ego_vehicle/vehicle_control_cmd");

    // PID controller parameters
    this->declare_parameter<double>("SpeedController.Kp", 0.);
    this->declare_parameter<double>("SpeedController.Ki", 0.);
    this->declare_parameter<double>("SpeedController.Kd", 0.);
    this->declare_parameter<double>("SpeedController.int_sat_min", -INF);
    this->declare_parameter<double>("SpeedController.int_sat_max", INF);
    this->declare_parameter<double>("SpeedController.sat_min", -INF);
    this->declare_parameter<double>("SpeedController.sat_max", INF);
}

void TwistToRegulatedControl::twistCallback(
    const geometry_msgs::msg::Twist &twist)
{
    rclcpp::Time time = this->get_clock()->now();

    auto carlaControl =
        std::make_unique<carla_msgs::msg::CarlaEgoVehicleControl>();
    int seconds = int(time.seconds());
    int nanoseconds = int(time.nanoseconds() - seconds * 1e9);
    carlaControl->header.stamp.sec = seconds;
    carlaControl->header.stamp.nanosec = nanoseconds;

    if (twist == geometry_msgs::msg::Twist())
    {
        // the command is to stop, brake
        carlaControl->brake = 1;
        _controlPublisher->publish(std::move(carlaControl));
        return;
    }

    // compute throttle control
    double refLinearSpeed = twist.linear.x;
    double throttle = computeThrottleControl(refLinearSpeed);

    // compute steering control
    double refAngularSpeed = twist.angular.z;
    double steering = computeSteeringControl(refAngularSpeed);

    // create the message to send
    carlaControl->throttle = throttle;
    carlaControl->steer =
        -steering; // need to invert the steering control for Carla

    _controlPublisher->publish(std::move(carlaControl));
}

void TwistToRegulatedControl::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    // get current simulation time
    double seconds = this->get_clock()->now().seconds();

    // get current linear speed
    _currentSpeed = odom.twist.twist.linear.x;

    // add the measure to the controller
    _speedController.addMeasure(_currentSpeed, seconds);
}

double
TwistToRegulatedControl::computeSteeringControl(double refAngularSpeed) const
{
    // ignore if the information are not initialized
    // TODO add a warning ?
    if (!_vehicleInfo.initialized)
        return 0.;

    // return 0 if the current speed is null
    if (_currentSpeed < SPEED_EPSILON)
        return 0.;

    // compute the wheel angle for the required angular speed
    double delta = std::atan(_vehicleInfo.L * refAngularSpeed / _currentSpeed);

    // normalize the value in [-1;1]
    double delta_carla = delta / _vehicleInfo.delta_max;

    return delta_carla;
}

double TwistToRegulatedControl::computeThrottleControl(double refLinearSpeed)
{
    double throttle = _speedController.computeControl(refLinearSpeed);
    return throttle;
}
