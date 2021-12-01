#ifndef PROJECT_DJI_SDK_H_H
#define PROJECT_DJI_SDK_H_H

namespace DJI {
    namespace OSDK {
        namespace Control{
    enum VerticalLogic
    {
        /*!
         - Set the control-mode to control the vertical
           speed of UAV, upward is positive
         - Limit: -5 to 5 m/s
         */
        VERTICAL_VELOCITY = 0x00,
        /*!
         - Set the control-mode to control the height of UAV
         - Limit: 0 to 120 m
         */
        VERTICAL_POSITION = 0x10,
        /*!
         - Set the control-mode to directly control the thrust
         - Range: 0% to 100%
         */
        VERTICAL_THRUST = 0x20,
    };

    /*! @brief bit 7:6 of the 8-bit (7:0) CtrlData.flag
     *
     *  @note
     *        - Only when the GPS signal is good (health_flag >=3)，horizontal
     * position control (HORIZONTAL_POSITION) related control modes can be used.
     *        - Only when GPS signal is good (health_flag >=3)，or when AdvancedSensing
     * system is working properly with Autopilot，
     *          horizontal velocity control（HORIZONTAL_VELOCITY）related control
     * modes can be used.
     */
    enum HorizontalLogic
    {
        /*!
         - Set the control-mode to control pitch & roll
         angle of the vehicle.
         - Need to be referenced to either the ground or
         body frame by HorizontalCoordinate setting.
         - Limit: 35 degree
         */
        HORIZONTAL_ANGLE = 0x00,
        /*!
         - Set the control-mode to control horizontal
         vehicle velocities.
         - Need to be referenced to either the ground
         or body frame by HorizontalCoordinate setting.
         - Limit: 30 m/s
         */
        HORIZONTAL_VELOCITY = 0x40,
        /*!
         - Set the control-mode to control position
         offsets of pitch & roll directions
         - Need to be referenced to either the ground
         or body frame by HorizontalCoordinate setting.
         - Limit: N/A
         */
        HORIZONTAL_POSITION = 0x80,
        /*!
         - Set the control-mode to control rate of
         change of the vehicle's attitude
         - Need to be referenced to either the ground
         or body frame by HorizontalCoordinate setting.
         - Limit: 150.0 deg/s
         */
        HORIZONTAL_ANGULAR_RATE = 0xC0
    };
    /*! @brief bit 3 of the 8-bit (7:0) CtrlData.flag
     */
    enum YawLogic
    {
        /*!
         - Set the control-mode to control yaw angle.
         - Yaw angle is referenced to the ground frame.
         - In this control mode, Ground frame is enforeced in Autopilot.
         */
        YAW_ANGLE = 0x00,
        /*!
         - Set the control-mode to control yaw angular velocity.
         - Same reference frame as YAW_ANGLE.
         - Limite: 150 deg/s
         */
        YAW_RATE = 0x08
    };

    /*! @brief bit 2:1 of the 8-bit (7:0) CtrlData.flag
     */
    enum HorizontalCoordinate
    {
        /*! Set the x-y of ground frame as the horizontal frame (NEU) */
        HORIZONTAL_GROUND = 0x00,
        /*! Set the x-y of body frame as the horizontal frame (FRU) */
        HORIZONTAL_BODY = 0x02
    };

    /*!
     * @brief bit 0 of the 8-bit (7:0) CtrlData.flag.
     *
     * Drone will try to hold at current position if enable
     */
    enum StableMode
    {
        STABLE_DISABLE = 0x00, /*!< Disable the stable mode */
        STABLE_ENABLE  = 0x01  /*!< Enable the stable mode */
    };

    /*!
     * @brief turn on or off the motors for emergency reasons
     */
    enum KillSwitch
    {
        ENABLE  = 0x01, /*!< Enable the killswitch */
        DISABLE = 0x02  /*!< Disable the killswitch */
    };
} } }

namespace DJISDK {

typedef enum AircraftVersion
{
  UNKNOWN,
  M100,
  M600,
  A3,
  N3,
  M210
} AircraftVersion;

enum FlightControlFlag
{
  HORIZONTAL_ANGLE         = DJI::OSDK::Control::HORIZONTAL_ANGLE,
  HORIZONTAL_VELOCITY      = DJI::OSDK::Control::HORIZONTAL_VELOCITY,
  HORIZONTAL_POSITION      = DJI::OSDK::Control::HORIZONTAL_POSITION,
  // Horizontal angular rate is supported only by A3/N3 based platform
  // and is NOT supported by M100
  HORIZONTAL_ANGULAR_RATE  = DJI::OSDK::Control::HORIZONTAL_ANGULAR_RATE,

  VERTICAL_VELOCITY = DJI::OSDK::Control::VERTICAL_VELOCITY,
  VERTICAL_POSITION = DJI::OSDK::Control::VERTICAL_POSITION,
  VERTICAL_THRUST   = DJI::OSDK::Control::VERTICAL_THRUST,

  YAW_ANGLE = DJI::OSDK::Control::YAW_ANGLE,
  YAW_RATE  = DJI::OSDK::Control::YAW_RATE,

  HORIZONTAL_GROUND = DJI::OSDK::Control::HORIZONTAL_GROUND,
  HORIZONTAL_BODY   = DJI::OSDK::Control::HORIZONTAL_BODY,

  STABLE_DISABLE = DJI::OSDK::Control::STABLE_DISABLE,
  STABLE_ENABLE  = DJI::OSDK::Control::STABLE_ENABLE
};

}

#endif //PROJECT_DJI_SDK_H_H
