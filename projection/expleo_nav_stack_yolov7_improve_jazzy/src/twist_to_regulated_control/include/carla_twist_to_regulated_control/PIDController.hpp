// =====================================================================================
//
//       Filename:  PIDController.hpp
//
//    Description:  PID controller class
//
//        Version:  1.0
//        Created:  11/10/2022 15:58:36
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Thibaud Dubautbout (TD), thibaud.duhautbout@expleogroup.com
//   Organization:  Expleo Group
//
// =====================================================================================
#ifndef CARLA_TWIST_TO_REGULATED_CONTROL__PID_CONTROLLER_HPP
#define CARLA_TWIST_TO_REGULATED_CONTROL__PID_CONTROLLER_HPP

#include <algorithm>
#include <limits>

constexpr double INF = std::numeric_limits<double>::infinity();

class PIDController
{
  private:
    double _Kp; //!< proportional gain
    double _Ki; //!< integral gain
    double _Kd; //!< derivative gain

    double _e;    //!< current error
    double _Ie;   //!< integral of the error
    double _De;   //!< derivative of the error
    double _vRef; //!< current reference
    double _t;    //!< current time

    double _Isat[2]; //!< saturations on the integral
    double _usat[2]; //!< saturations on the output

  public:
    /**
     * \brief Standard constructor
     * \param[in] Kp proportional gain
     * \param[in] Ki integral gain
     * \param[in] Kd derivative gain
     * \param[in] int_sat_min minimal saturation for the integrator
     * \param[in] int_sat_max maximal saturation for the integrator
     * \param[in] u_sat_min minimal saturation for the output
     * \param[in] u_sat_max maximal saturation for the output
     *
     * The constructor takes the different gains of the regulator as parameters.
     */
    PIDController(double Kp = 0,
                  double Ki = 0,
                  double Kd = 0,
                  double int_sat_min = -INF,
                  double int_sat_max = INF,
                  double u_sat_min = -INF,
                  double u_sat_max = INF);

    /// Update the value of the proportional gain
    inline void setKp(double Kp) { _Kp = Kp; }

    /// Update the value of the integral gain
    inline void setKi(double Ki) { _Ki = Ki; }

    /// update the value of the derivative gain
    inline void setKd(double Kd) { _Kd = Kd; }

    /// update the integrator minimal saturation
    inline void setIntSatMin(double sat) { _Isat[0] = sat; }

    /// update the integrator maximal saturation
    inline void setIntSatMax(double sat) { _Isat[1] = sat; }

    /// update the output minimal saturation
    inline void setSatMin(double sat) { _usat[0] = sat; }

    /// update the output maximal saturation
    inline void setSatMax(double sat) { _usat[1] = sat; }

    void addMeasure(double v, double t);

    /**
     * \brief Compute the control value to apply
     * \param[in] v the current value of the system to control
     * \param[in] v_ref the reference value to which the system must be
     * controlled \param[in] t the current time value (in seconds)
     *
     * Compute the control value \f$u\f$ to apply based on the current value
     * \f$v\f$ and the reference value \f$v_{ref}\f$ according to the PID
     * formula:
     *
     * \f[
     *    e(t) = v(t) - v_{ref}(t) \\
     *    u(t) = K_p \times e(t) + K_i \times \int_0^t e(w)dw + K_d \times
     * \dot{e}(t)
     * \f]
     */
    double computeControl(double v_ref);

    /// reset the integrator of the controller
    inline void reset() { _Ie = 0; }
};

#endif // #ifndef CARLA_TWIST_TO_REGULATED_CONTROL__PID_CONTROLLER_HPP
