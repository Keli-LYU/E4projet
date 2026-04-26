// =====================================================================================
//
//       Filename:  TwistToRegulatedControl.hpp
//
//    Description:  Node to convert twist messages to carla control messages
//
//        Version:  1.0
//        Created:  11/10/2022 17:18:25
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Thibaud Dubautbout (TD), thibaud.duhautbout@expleogroup.com
//   Organization:  Expleo Group
//
// =====================================================================================
#ifndef CARLA_TWIST_TO_REGULATED_CONTROL__TWIST_TO_REGULATED_CONTROL_HPP
#define CARLA_TWIST_TO_REGULATED_CONTROL__TWIST_TO_REGULATED_CONTROL_HPP

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "carla_twist_to_regulated_control/PIDController.hpp"

constexpr double SPEED_EPSILON = 1e-2;

class VehicleInfo
{
  public:
    double delta_max = 0;
    double L = 0;
    bool initialized = false;
};

class TwistToRegulatedControl : public rclcpp::Node
{
  private:
    PIDController _speedController; //!< PID controller for the speed

    /// subscriber on the twist topic
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twistSubscriber;

    /// subscriber on the odometry topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;

    /// publisher of Carla control messages
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
        _controlPublisher;

    double _currentSpeed;     //!< current linear speed of the vehicle
    VehicleInfo _vehicleInfo; //!< information about the vehicle

    /// declare the parameters used by the node
    void declare_node_parameters();

    /**
     * \brief Compute the steering control to apply
     * \param[in] refAngularSpeed target angular speed of the vehicle
     * \return \f$\delta_{carla}\f$ the steering control command for Carla
     *
     * This method computes the steering input based on a kinematic bycicle
     * model. According to this model:
     *
     * \f[
     *   \dot{\theta} = \frac{v}{L} \times \tan\left(\delta\right)
     * \f]
     *
     * with:
     * - \f$\dot\theta\f$ the angular speed of the vehicle
     * - \f$v\f$ the linear speed of the vehicle
     * - \f$L\f$ the longitudinal length between the wheels
     * - \f$\delta\f$ the steering angle of the wheels
     *
     * This method computes the steering angle by reversing this formula:
     *
     * \f[
     *   \delta = \arctan\left(\frac{L\times \dot\theta}{v}\right)
     * \f]
     *
     * If \f$v<\varepsilon\f$, the method returns \f$\delta=0\f$.
     * The \f$\arctan\f$ function returns a value between
     * \f$\left[-\frac{\pi}{2};\frac{\pi}{2}\right]\f$ so there is no problem
     * for the wheel steering angle which is usually inside this interval.
     *
     * The steering command for Carla must be in \f$[-1;1]\f$, so the resulting
     * steering value is computed into this interval based on the maximum
     * steering angle \f$\delta_{max}\f$:
     *
     * \f[
     *  \delta_{carla} = \frac{\delta}{\delta_{max}}
     * \f]
     */
    double computeSteeringControl(double refAngularSpeed) const;

    double computeThrottleControl(double refLinearSpeed);

  public:
    /// constructor
    TwistToRegulatedControl();

    void twistCallback(const geometry_msgs::msg::Twist &twist);
    void odomCallback(const nav_msgs::msg::Odometry &twist);
};

#endif // #ifndef
       // CARLA_TWIST_TO_REGULATED_CONTROL__TWIST_TO_REGULATED_CONTROL_HPP
