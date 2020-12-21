//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Carlos Pinto                                                     *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  //! PID Controllers for Heading and Speed. 
  //! Highly based on Ricardo Gomes' and José Braga's control task 
  //! (Control/ASV/HeadingAndSpeed). 
  //! Task subscribes to messages that specify desired heading and distance 
  //! to destination. It then generates an output value for thrusters and 
  //! rummers in order to minimize error. Output value range is parameter 
  //! defined.
  //! 
  //! @author Carlos Pinto
  namespace USV
  {
    namespace SimpleHeadingAndSpeed
    {

      using DUNE_NAMESPACES;

      static const fp64_t c_forward_cone = Math::c_pi/4.0;
      static const fp64_t c_forward_cone_tol = Math::c_pi/36.0;

      struct Arguments
      {
        //! Maximum Thruster Output
        float max_thruster_act;
        //! Maximum Thruster Differential Output
        float max_thruster_diff;
        //! Maximum Thruster Change Rate
        float max_thruster_rate;
        //! Percentage of thruster usage to change heading when at low speed
        float low_spd_thrust_percent;
        //! Percentage of rudder usage to change heading when at high speed
        float high_spd_rudder_percent;
        //! Threshold that defines "Moving Fast" and "Moving Slowly"
        float speed_threshold;
        //! Hysteresis distance betwen "Moving Fast" and "Moving Slowly"
        float speed_hysteresis;
        //! PID gains for Speed controller
        std::vector<float> speed_gains;
        //! PID gains for Heading controller
        std::vector<float> heading_gains;
        //! Entity label of port motor
        std::string entityid_motor_port;
        //! Entity label of starboard motor
        std::string entityid_motor_starboard;
        //! Entity label of rudder
        std::string entityid_rudder;
      };

      struct Task: public DUNE::Tasks::Task
      {
	double m_time_last_state;
        //! Entity ID of motor port based on Entity Label Resolution
        uint16_t m_entity_id_motor_port;
        //! Entity ID of motor starboard based on Entity Label Resolution
        uint16_t m_entity_id_motor_starboard;
        //! Entity ID of rudder based on Entity Label Resolution
        uint16_t m_entity_id_rudder;
        //! Target speed which speed PID will attempt to achieve
        float m_target_speed;
        //! Target heading which heading PID will attempt to achieve
        float m_target_heading;
        //! partial actuation value to send to the thruster, in order to change heading
        float m_thruster_act_heading;
        //! actual actuation value to send to the starboard thruster
        float m_thruster_act_starboard;
        //! actual actuation value to send to the port thruster
        float m_thruster_act_port;
        //! previous actuation value sent to the starboard thruster
        float m_thruster_prev_act_starboard;
        //! previous actuation value sent to the port thruster
        float m_thruster_prev_act_port;
        //! actuation value to send to rudders, high values mean "turn to starboard"
        float m_rudders_act;
        //! Vessel is moving at high speed
        bool m_high_speed;
        //! Vessel is moving forward
        bool m_moving_to_dest;
        //! Speed PID controller
        DUNE::Control::DiscretePID m_speed_pid;
        //! Heading PID controller
        DUNE::Control::DiscretePID m_heading_pid;
        //! Time between estimated states used in PID controllers
        DUNE::Time::Delta m_timestep;
        //! Control loops last reference
        uint32_t m_scope_ref;
        //! Task arguments (parameters)
        Arguments m_args;
        
        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx)
        {
          param("Maximum Thruster Output",m_args.max_thruster_act)
          .defaultValue("1.0")
          .description("Maximum value that can be sent to each Motor");

          param("Maximum Thruster Differential", m_args.max_thruster_diff)
          .defaultValue("0.2")
          .description("Maximum value difference between each Motor");

          param("Maximum Thruster Rate", m_args.max_thruster_rate)
          .defaultValue("0.0")
          .description("Maximum value of thruster actuation change in Duty Cycle/second");

          param("Low Speed Thruster Percentage", m_args.low_spd_thrust_percent)
          .defaultValue("0.2")
          .description("Percentage to apply to the thrusters when at low speeds");

          param("High Speed Rudder Percentage", m_args.high_spd_rudder_percent)
          .defaultValue("0.8")
          .description("Percentage to apply to the thrusters when at high speeds");

          param("High Speed Threshold", m_args.speed_threshold)
          .defaultValue("1.0")
          .description("Speed threshold that limits High Speed and Low Speed");

          param("High Speed Hysteresis", m_args.speed_hysteresis)
          .defaultValue("0.1")
          .description("Hysteresis distance between average threshold and upper and lower limits");
          
          param("Speed PID Gains", m_args.speed_gains)
          .defaultValue("")
          .size(3)
          .description("PID gains for Speed Controller, given as Kp Ki Kd, respectively");
          
          param("Heading PID Gains", m_args.heading_gains)
          .defaultValue("")
          .size(3)
          .description("PID gains for Heading Controller, given as Kp Ki Kd, respectively");

          param("Entity Label - Port Motor", m_args.entityid_motor_port)
          .defaultValue("Motor - Port")
          .description("Entity label of port motor rpm");

          param("Entity Label - Starboard Motor", m_args.entityid_motor_starboard)
          .defaultValue("Motor - Starboard")
          .description("Entity label of starboard motor rpm");

          param("Entity Label - Rudder", m_args.entityid_rudder)
          .defaultValue("Rudder")
          .description("Entity label of rudders");
        
          // Start system at a standstill
          m_target_speed = 0.0;
          m_high_speed = false;
          m_moving_to_dest = false;
          m_thruster_prev_act_port = 0.0;
          m_thruster_prev_act_starboard = 0.0;
	  m_time_last_state = 0;

          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);

          war("Startup: controller disabled");

          // Subscribe to relevant IMC messages
          bind<IMC::ControlLoops>(this);
          bind<IMC::Abort>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::DesiredSpeed>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.speed_gains) ||
              paramChanged(m_args.heading_gains))
          {
            resetPID();
            setupPID();
          }

          if (paramChanged(m_args.max_thruster_rate))
          {
            m_args.max_thruster_rate /= 100; // convert from % to fraction
          }
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
        }

        //! Resolve entities.
        void
        onEntityResolution(void)
        {
          // This is just so we don't use strings to define entities,
          // and use instead an associated id. 
          // THIS ISN'T BEING USED YET!
          try
          {
            m_entity_id_motor_port = resolveEntity(m_args.entityid_motor_port);
          }
          catch (...)
          {
            m_entity_id_motor_port = 0xffff;
          }

          try
          {
            m_entity_id_motor_starboard = resolveEntity(m_args.entityid_motor_starboard);
          }
          catch (...)
          {
            m_entity_id_motor_starboard = 0xffff;
          }

          try
          {
            m_entity_id_rudder = resolveEntity(m_args.entityid_rudder);
          }
          catch (...)
          {
            m_entity_id_rudder = 0xffff;
          }
        }

        //! On activation
        void
        onActivation(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }

        //! On deactivation
        void
        onDeactivation(void)
        {
          stop();
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        }

        //! Acquire resources.
        void
        onResourceAcquisition(void)
        {
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);
          }
        }

        void
        consume (const IMC::ControlLoops* msg)
        {
          if (!(msg->mask & (IMC::CL_YAW | IMC::CL_SPEED)))
            return;

          if (msg->scope_ref < m_scope_ref)
            return;

          m_scope_ref = msg->scope_ref;

          if (msg->enable == isActive())
            return;

          if (msg->enable)
            requestActivation();
          else
            requestDeactivation();

          war(isActive() ? DTR("enabling") : DTR("disabling"));

          if (!isActive())
            resetPID();
        }

        void
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          // This works as redundancy, in case everything else fails
          resetPID();
          war("disabling");
        }

        void
        consume(const IMC::DesiredHeading *msg)
        {
          m_target_heading = msg->value;
        }

        void
        consume(const IMC::DesiredSpeed *msg)
        {
          m_target_speed = msg->value;
        }

        void
        consume(const IMC::EstimatedState *msg){
          
          if (msg->getSource() != getSystemId())
            return;

          if (!isActive())
          {
            m_target_heading = msg->psi;
            m_target_speed = msg->u;
            return;
          }

          double time_step = m_timestep.getDelta();
	  m_time_last_state += time_step;
          if (time_step <= 0)
          {
            return;
          }
	  m_time_last_state = 0;

          float current_direction;
          float current_speed = msg->u;
          evaluateSpeed(current_speed);
          if (m_high_speed)
          {
            current_direction = msg->psi;
          }
          else
          {
            current_direction = msg->psi; // for now, until i don't know how to get true heading
            //desired_direction = msg->TRUE_HEADING;
          }

          float heading_error = Angles::normalizeRadian(m_target_heading - current_direction);
          float heading_output = m_heading_pid.step(time_step, heading_error);
          movingForward(heading_error);
          
          float speed_error = 0;
          float speed_output = 0;

          if (m_moving_to_dest) // only use speed control after we're moving towards destination point
          {
            speed_error = m_target_speed - current_speed;
            speed_output = m_speed_pid.step(time_step, speed_error);
            spew("New Estimated state received by Controller! Heading error is %f rad and speed error is %f knots", heading_error, speed_error);
          }
          else
          {
            spew("New Estimated state received by Controller! Heading error is %f rad and is too big, so no speed PID will be used", heading_error);
          }

          distributeHeadingControl(heading_output);
          distributeThrusterControl(speed_output);

          limitThrusterRate(time_step);

          m_thruster_act_port = normalize (m_thruster_act_port, -m_args.max_thruster_act, m_args.max_thruster_act);
          m_thruster_act_starboard = normalize (m_thruster_act_starboard, -m_args.max_thruster_act, m_args.max_thruster_act);

          sendActuation();
        }

        //! Reset PIDs. 
        //! Resets error memory from PIDs (last error and integral of error)
        void
        resetPID(void)
        {
          m_speed_pid.reset();
          m_heading_pid.reset();

          m_target_speed = 0.0;
          m_high_speed = false;
          m_moving_to_dest = false;
          m_thruster_prev_act_port = 0.0;
          m_thruster_prev_act_starboard = 0.0;
          
          stop();
        }

        //! Sends standby signals to thrusters and rudders
        void
        stop(void)
        {
          IMC::SetThrusterActuation thrust_act_starboard;
          IMC::SetThrusterActuation thrust_act_port;
          IMC::SetServoPosition rudder;

          thrust_act_starboard.id = 0;
          thrust_act_starboard.value = 0.0;
          thrust_act_port.id = 1;
          thrust_act_port.value = 0.0;
          rudder.id = 1;
          rudder.value = 0.0;

          dispatch(thrust_act_starboard);
          dispatch(thrust_act_port);
          dispatch(rudder);
        }

        //! Sets up PID controllers. 
        //! Sets up PID gains and output limits based on specified parameters
        void
        setupPID(void)
        {
          m_speed_pid.setGains(m_args.speed_gains);
          m_speed_pid.setOutputLimits(-m_args.max_thruster_act, m_args.max_thruster_act);

          m_heading_pid.setGains(m_args.heading_gains);
          m_heading_pid.setOutputLimits(-m_args.max_thruster_act, m_args.max_thruster_act);
        }

        //! Distributes Heading control signals. 
        //! Based on current speed, distributes the heading control between
        //! thrusters and rudders
        //! @param[in] heading_output desired output to control heading, negative means "Turn to Port"
        //! @param[in] speed speed at which the vessel is currently moving
        void
        distributeHeadingControl(float heading_output)
        {
          // calculate distribution according to current speed state
          if (m_high_speed)
          {
            m_thruster_act_heading = heading_output;
            if (m_moving_to_dest)//TO DO: this is based on SOG, which might not coincide (and usually doesn't) with speed in relation to water, which is what matters for rudders to turn the vessel
            { //using rudders only makes sense if we're moving forward 
              m_rudders_act = heading_output * m_args.high_spd_rudder_percent;
            }
          }
          else
          {
            m_thruster_act_heading = heading_output * m_args.low_spd_thrust_percent;
            m_rudders_act = heading_output;
          }

          // guarantee that differential motor control is inside specified difference
          // m_thruster_act_heading = DUNE::Math::trimValue(m_thruster_act_heading, -m_args.max_thruster_diff, m_args.max_thruster_diff);
          // // normalize rudders actuation between 0 and 1
          // m_rudders_act = normalize(m_rudders_act, -m_args.max_thruster_act, m_args.max_thruster_act);
        }

        //! Distributes thruster control values. 
        //! Based on thruster actuation values that result from both 
        //! heading and speed control, merge both to get the effective 
        //! actuation value to be applied to both thrusters and make 
        //! sure they aren't bigger than max actuation values
        //! @param[in] speed_output desired output for speed control (i.e. common actuation)
        void
        distributeThrusterControl(float speed_output)
        {
          // distribute actuation
          m_thruster_act_starboard = speed_output - m_thruster_act_heading;
          m_thruster_act_port = speed_output + m_thruster_act_heading;

          // check if no limits were transpassed; favor heading control if they were.
          // note that due to the structure of the control only one of these conditions can apply at any time
          if (m_thruster_act_starboard > m_args.max_thruster_act) // we want to turn to port "a lot" - starboard thruster
          {
            m_thruster_act_port -= (m_thruster_act_starboard-m_args.max_thruster_act);
            m_thruster_act_starboard = m_args.max_thruster_act;
          }
          else if (m_thruster_act_starboard < -m_args.max_thruster_act) // we want to turn to starboard "a lot" - starboard thruster
          {
            m_thruster_act_port -= (m_thruster_act_starboard + m_args.max_thruster_act);
            m_thruster_act_starboard = -m_args.max_thruster_act;
          }

          else if (m_thruster_act_port > m_args.max_thruster_act) // we want to turn to starboard "a lot" - port thruster
          {
            m_thruster_act_starboard -= (m_thruster_act_port-m_args.max_thruster_act);
            m_thruster_act_port = m_args.max_thruster_act;
          }
          else if (m_thruster_act_port < -m_args.max_thruster_act) // we want to turn to port "a lot" - port thruster
          {
            m_thruster_act_starboard -= (m_thruster_act_port + m_args.max_thruster_act);
            m_thruster_act_port = -m_args.max_thruster_act;
          }
        }

        //! Classifies current speed state. 
        //!
        //! Based on current speed, decides if speed is currently fast or slow 
        //! using a simple threshold compare with hysteresis
        //! @param[in] speed speed at which we're currently moving
        void
        evaluateSpeed(float speed)
        {
          if ((!m_high_speed) && 
              (speed > m_args.speed_threshold + (m_args.speed_hysteresis/2.0)))
          {
            m_high_speed = true;
          }
          else if ((m_high_speed) &&
                   (speed < m_args.speed_threshold - (m_args.speed_hysteresis/2.0)))
          {
            m_high_speed = false;
          }
        }

        //! Normalize value. 
        //! Normalizes value to fit on a scale between 0 and 1, 
        //! from an original scale specified as a parameter
        //! @param[in] value value to be normalized
        //! @param[in] min minimum value of original scale
        //! @param[in] max maximum value of original scale
        //! @return normalized value between 0 and 1
        float
        normalize (float value, float min, float max){
          value = DUNE::Math::trimValue(value, min, max);
          return (value-min)/(max-min);
        }

        //! Limit Thruster Rate of Change. 
        //! Limits the rate between each consecutive thruster actuation
        //! in order to avoid sudden acceleration/braking
        //! @param[in] timestep time occurred between this actuation and the last
        void
        limitThrusterRate(double timestep)
        {
          if (m_args.max_thruster_rate == 0.0){
            return;
          }
          m_thruster_act_port = limitDerivative(timestep, m_thruster_act_port, m_thruster_prev_act_port, m_args.max_thruster_rate);
          m_thruster_act_starboard = limitDerivative(timestep, m_thruster_act_starboard, m_thruster_prev_act_starboard, m_args.max_thruster_rate);
          m_thruster_prev_act_port = m_thruster_act_port;
          m_thruster_prev_act_starboard = m_thruster_act_starboard;
        }

        //! Limit derivative of a value. 
        //! Limits the derivative of a value, in respect to a provided max derivative
        //! @param[in] timestep time occurred between this value and the previous
        //! @param[in] current_value current value whose derivative will be limited
        //! @param[in] previous_value previous value to be used in order to compute derivative
        //! @param[in] maximum derivative (absolute value)
        //! @return limited value
        float
        limitDerivative(double timestep, float current_value, float previous_value, float max_derivative)
        {
          if (timestep == 0) // infinite derivative, ignore limitation
          {
            return current_value;
          }
          else
          {
            float derivative = (current_value - previous_value) / timestep;
            float max_allowed_change = previous_value + (max_derivative * timestep);

            if (derivative > 0)
            {
              current_value = previous_value + DUNE::Math::trimValue (derivative, 0.0, max_allowed_change);
            }
            else
            {
              current_value = previous_value + DUNE::Math::trimValue (derivative, -max_allowed_change, 0.0);
            }
          }
          return current_value;
        }

        void
        movingForward(float heading_error)
        {
          if (!m_moving_to_dest)
          {
            if (std::fabs(heading_error) < c_forward_cone)
            {
              m_moving_to_dest = true;
            }
          }
          else
          {
            if (std::fabs(heading_error) > c_forward_cone + c_forward_cone_tol)
            {
              m_moving_to_dest = false;
            }
          }
        }

        void
        sendActuation(void)
        {
          IMC::SetThrusterActuation thruster_port, thruster_starboard;
          IMC::SetServoPosition rudders;

          thruster_port.value = m_thruster_act_port;
          thruster_port.id = 0;
          thruster_starboard.value = m_thruster_act_starboard;
          thruster_starboard.id = 1;
          rudders.value = m_rudders_act;
          rudders.id = 1;

          dispatch(thruster_port);
          dispatch(thruster_starboard);
          dispatch(rudders);
          spew ("Control signals dispatched:\n\tPort Thruster: %f\n\tStarboard Thruster %f\n\tRudders: %f", thruster_port.value, thruster_starboard.value, rudders.value);
        }
      };
    }
  }
}

DUNE_TASK
