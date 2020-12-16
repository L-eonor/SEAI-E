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



//! Rad tolerance to be considered zero.
static const float c_low_rad = 1.0f;
//! Actuation tolerance to be considered zero.
static const float c_low_act = 0.1f;

namespace Simulators
{
  //! %Rudder simulator for DUNE
  //!
  //! %Highly based on Ricardo Martins' Motor
  //! simulator task (Simulators/Motor). 
  //! Gathers actuation values for servos (rudders). 
  //! Outputs servo position in radians.
  //! 
  //! @author Carlos Pinto
  namespace USV
  {
    namespace Rudders
    {

    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      //! Number of samples to average.
      int avg_samples;
      //! Parameters to convert servo actuation to rudders position in radian.
      std::vector<double> act_to_rad_args;
      //! Rudder Id.
      int rudder_id;
    };

    //! %Motor simulator task
    struct Task: public Tasks::Periodic
    {
      //! Moving average filter for frequency of rotation.
      MovingAverage<double>* m_avg_rudder;
      //! Filtered servo position (rad).
      IMC::ServoPosition m_servo_pos;
      //! New frequency of rotation value.
      fp32_t m_rad_new;
      //! Task arguments.
      Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Periodic(name, ctx),
        m_avg_rudder(NULL)
      {
        // Retrieve configuration values.
        param("Moving Average Samples", m_args.avg_samples)
        .defaultValue("5")
        .description("Number of moving average samples to average radian position");

        param("Rudder Act to Radian Factor", m_args.act_to_rad_args)
        .defaultValue("")
        .size(1)
        .description("Parameters to convert rudder actuation (%) to position (rad)");

        // Initialize Rudder Id.
        param("Rudder Id", m_args.rudder_id)
        .defaultValue("0")
        .description("Rudder identification");

        // Register consumers.
        bind<IMC::SetServoPosition>(this);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_avg_rudder);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Initialize Servo values.
        m_servo_pos.value = 0;
        m_rad_new = 0;
        m_avg_rudder = new MovingAverage<double>(m_args.avg_samples);

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      consume(const IMC::SetServoPosition* msg)
      {
        // A simple model of ServoPosition = a * act is used
        // Due to lack of access to real test systems, this value
        // serves as a placeholder value to be replaced
        // by an empirically obtained value later on
        if (msg->id == m_args.rudder_id)
        {
          m_rad_new = (fp32_t)(m_args.act_to_rad_args[0] * msg->value);

          // // Below is an implementation using a linear model with offset such as:
          // // ServoPosition = a + b * act (where a has the same signal as act in nay case)
          // // change size of m_args.act_to_rad_args vector to 2 in order to use code below
          // if (std::abs(msg->value) >= c_low_act)
          //   m_rad_new = (fp32_t)(m_args.act_to_rad_args[0] * (msg->value / std::abs(msg->value)) + m_args.act_to_rad_args[1] * msg->value);
          // else
          //   m_rad_new = 0;
        }
      }

      void
      task(void)
      {
        // Compute filtered radian value.
        // This value is computed using a moving average filter.
        m_servo_pos.value = (fp32_t)Math::round(m_avg_rudder->update(m_rad_new));

        // Threshold check.
        if (std::abs(m_servo_pos.value) < c_low_rad)
          m_servo_pos.value = 0;

        // Send to bus.
        dispatch(m_servo_pos);
      }
    };
    }
  }
}

DUNE_TASK
