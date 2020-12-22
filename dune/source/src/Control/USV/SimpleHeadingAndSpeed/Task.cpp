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
  //! %PID Controllers for Heading and Speed
  //! %Task subscribes to messages that specify desired heading and distance
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

      struct Arguments
      {
        //! Maximum Thruster Output
        float max_thruster_act;
        //! Maximum Thruster Differential Output
        float max_thruster_diff;
        //! Maximum Thruster Change Rate
        float max_thruster_rate;
        //! Percentage of thruster usage to change heading when at high speed
        float low_spd_thrust_percent;
        //! Percentage of thruster usage to change heading when at low speed
        float high_spd_thrust_percent;
        //! Threshold that defines "Moving Fast" and "Moving Slowly"
        float speed_threshold;
        //! Hysteresis distance betwen "Moving Fast" and "Moving Slowly"
        float speed_hysteresis;
        //! PID gains for Speed controller
        std::vector<float> speed_gains;
        //! PID gains for Heading controller
        std::vector<float> heading_gains;
      };

      struct Task: public DUNE::Tasks::Task
      {

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
        //! Speed PID controller
        DUNE::Control::DiscretePID m_speed_pid;
        //! Heading PID controller
        DUNE::Control::DiscretePID m_heading_pid;
        //! Time between estimated states used in PID controllers
        DUNE::Time::Delta m_timestep;
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
          .description("Maximum value between each consecutive actuation per second");

          param("Low Speed Thruster Percentage", m_args.low_spd_thrust_percent)
          .defaultValue("0.2")
          .description("Percentage to apply to the thrusters when at low speeds");

          param("High Speed Thruster Percentage", m_args.high_spd_thrust_percent)
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
        
          // Start system at a standstill
          m_target_speed = 0.0;
          m_high_speed = false;
          m_thruster_prev_act_port = 0.0;
          m_thruster_prev_act_starboard = 0.0;

          // Subscribe to relevant IMC messages
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
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
        }

        //! Resolve entity names.
        void
        onEntityResolution(void)
        {
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
          double time_step = m_timestep.getDelta();
          if (time_step < 0.0)
          {
            return;
          }

          float heading_error = Angles::normalizeRadian(m_target_heading - msg->psi);
          float heading_output = m_heading_pid.step(time_step, heading_error);

          float speed_error = m_target_speed - msg->u;
          float speed_output = m_speed_pid.step(time_step, speed_error);

          spew("New Estimated state received by Controller! Heading error is %f rad and speed error is %f knots", heading_error, speed_error);

          distributeHeadingControl(heading_output, msg->u);
          distributeThrusterControl(speed_output);

          limitThrusterJitter(time_step);

          m_thruster_act_port = normalize (m_thruster_act_port, -m_args.max_thruster_act, m_args.max_thruster_act);
          m_thruster_act_starboard = normalize (m_thruster_act_starboard, -m_args.max_thruster_act, m_args.max_thruster_act);

          sendActuation();
        }

        //! %Reset PIDs
        //! %Resets error memory from PIDs (last error and integral of error)
        void
        resetPID(void)
        {
          m_speed_pid.reset();
          m_heading_pid.reset();
        }

        //! %Sets up PID controllers
        //! %Sets up PID gains and output limits based on specified parameters
        void
        setupPID(void)
        {
          m_speed_pid.setGains(m_args.speed_gains);
          m_speed_pid.setOutputLimits(-m_args.max_thruster_act, m_args.max_thruster_act);

          m_heading_pid.setGains(m_args.heading_gains);
          m_heading_pid.setOutputLimits(-m_args.max_thruster_act, m_args.max_thruster_act);
        }

        //! %Distributes Heading control signals
        //! %Based on current speed, distributes the heading control between
        //! thrusters and rudders
        //! @param[in] heading_output desired output to control heading, negative means "Turn to Port"
        //! @param[in] speed speed at which the vessel is currently moving
        void
        distributeHeadingControl(float heading_output, float speed)
        {
          // determine if moving fast or slow
          evaluateSpeed(speed);

          // calculate distribution according to current speed state
          if (m_high_speed)
          {
            m_thruster_act_heading = heading_output * m_args.high_spd_thrust_percent;
            m_rudders_act = heading_output * (1-m_args.high_spd_thrust_percent);
          }
          else
          {
            m_thruster_act_heading = heading_output * m_args.low_spd_thrust_percent;
            m_rudders_act = heading_output * (1-m_args.low_spd_thrust_percent);
          }

          // guarantee that differential motor control is inside specified difference
          m_thruster_act_heading = DUNE::Math::trimValue(m_thruster_act_heading, -m_args.max_thruster_diff, m_args.max_thruster_diff);
          // normalize rudders actuation between 0 and 1
          m_rudders_act = normalize(m_rudders_act, -m_args.max_thruster_act, m_args.max_thruster_act);
        }

        //! %Distributes thruster control values
        //! %Based on thruster actuation values that result from both
        //! heading and speed control, merge both to get the effective
        //! actuation value to be applied to both thrusters
        //! @param[in] speed_output desired output to control speed
        void
        distributeThrusterControl(float speed_output)
        {
          // distribute actuation
          m_thruster_act_starboard = speed_output - m_thruster_act_heading/2.0;
          m_thruster_act_port = speed_output + m_thruster_act_heading/2.0;

          // check if no limits were transpassed; favor heading control if they were.
          if (m_thruster_act_starboard > m_args.max_thruster_act) // we want to turn to starboard "a lot"
          {
            m_thruster_act_port = m_args.max_thruster_act + m_thruster_act_heading;
            m_thruster_act_starboard = m_args.max_thruster_act;
          }
          else if (m_thruster_act_port < -m_args.max_thruster_act) // we want to turn to port "a lot"
          {
            m_thruster_act_port = -m_args.max_thruster_act;
            m_thruster_act_starboard = -m_args.max_thruster_act - m_thruster_act_heading;
          }
        }

        //! %Classifies current speed state
        //! %Based on current speed, decides if speed is currently fast or slow
        //! @param[in] speed speed at which we're currently moving
        void
        evaluateSpeed(float speed)
        {
          if ((speed > m_args.speed_threshold + (m_args.speed_hysteresis/2)) && (!m_high_speed))
          {
            m_high_speed = true;
          }
          else if ((speed < m_args.speed_threshold - (m_args.speed_hysteresis/2)) && (m_high_speed))
          {
            m_high_speed = false;
          }
        }

        //! %Normalize value
        //! %Normalizes value to fit on a scale between 0 and 1, 
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

        //! %Limit Thruster Rate of Change
        //! %Limits the rate between each consecutive thruster actuation
        //! in order to avoid sudden acceleration/braking
        //! @param[in] timestep time occurred between this actuation and the last
        void
        limitThrusterJitter(double timestep)
        {
          if (m_args.max_thruster_rate == 0.0){
            return;
          }
          m_thruster_act_port = limitDerivative(timestep, m_thruster_act_port, m_thruster_prev_act_port, m_args.max_thruster_rate);
          m_thruster_act_starboard = limitDerivative(timestep, m_thruster_act_starboard, m_thruster_prev_act_starboard, m_args.max_thruster_rate);
          m_thruster_prev_act_port = m_thruster_act_port;
          m_thruster_prev_act_starboard = m_thruster_act_starboard;
        }

        //! %Limit derivative of a value
        //! %Limits the derivative of a value, in respect to a provided max derivative
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
        sendActuation(void)
        {
          IMC::SetThrusterActuation thruster_port, thruster_starboard;
          IMC::SetServoPosition rudders;

          thruster_port.value = m_thruster_act_port;
          thruster_port.id = 1;
          thruster_starboard.value = m_thruster_act_starboard;
          thruster_starboard.id = 2;
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
