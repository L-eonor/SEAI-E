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
// Author: Francisco                                                        *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <cstdlib>
#include <time.h>  



namespace Actuators
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Francisco
  namespace Value_Generator
  {
    using DUNE_NAMESPACES;

    //!Task arguments.
    struct Arguments
    {
      //! Zig Zag value generation.
      bool zigzag;
      //! Dispatch frequency.
      double freq;
      //! Increment for every cycle.
      float increment;
      //! Upper bound of generated values.
      float upper_bound;
      //! Lower bound of generated values.
      float lower_bound;
    };
    
    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments.
      Arguments m_args;
      //! Current value to send to thruster 1
      float m_t1_curr_value;
      //! Current value to send to thruster 2
      float m_t2_curr_value;
      //! Current value to send to rudder 
      float m_s1_curr_value;
      //! Weather we're going up or down in zig zag mode
      bool m_up;
      //! Actuation Message to send to thruster
      IMC::SetThrusterActuation m_thruster_1, m_thruster_2;
      //! Actuation Message to send to rudder
      IMC::SetServoPosition m_servo_1;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
        //m_prng(NULL)
      {
        param("Zig Zag Mode", m_args.zigzag)
        .description("Weather to use RNG values or up and down values")
        .defaultValue("true");

        param("Dispatch Frequency", m_args.freq)
        .description("Frequency at which task runs.")
        .defaultValue("1.0");

        param("Value Increment", m_args.increment)
        .description("Value at which generated values increase in zig zag mode")
        .defaultValue("0.1");

        param("Upper Bound", m_args.upper_bound)
        .description("Upper bound of generated values")
        .defaultValue("1.0");

        param("Lower Bound", m_args.lower_bound)
        .description("Upper bound of generated values")
        .defaultValue("-1.0");


        m_thruster_1.setSourceEntity(getEntityId());
        m_thruster_1.id = 0;
        m_thruster_2.setSourceEntity(getEntityId());
        m_thruster_2.id = 1;
        m_servo_1.setSourceEntity(getEntityId());
        m_servo_1.id = 1;
      }

      void
      onEntityReservation(void)
      {
        setFrequency(m_args.freq);
        m_t1_curr_value = m_args.upper_bound;
        m_t2_curr_value = m_args.upper_bound;
        m_s1_curr_value = m_args.upper_bound;
      }

      //! Aquire resources.
      void
      onResourceAcquisition(void)
      {

      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

      }


      //! Periodic work.
      void
      task(void)
      {
        uint8_t msg_num;
        int value;
  
        //Zig Zag mode
        if (m_args.zigzag)
        {
          if (m_up)
          {
            m_t1_curr_value += m_args.increment;
            m_t2_curr_value += m_args.increment;
            m_s1_curr_value += m_args.increment;

            if (m_t1_curr_value > m_args.upper_bound)
            {
              m_up = false;
              m_t1_curr_value -= 2*m_args.increment;
              m_t2_curr_value -= 2*m_args.increment;
              m_s1_curr_value -= 2*m_args.increment;
            }
          }
          else
          {
            m_t1_curr_value -= m_args.increment;
            m_t2_curr_value -= m_args.increment;
            m_s1_curr_value -= m_args.increment;

            if (m_t1_curr_value < m_args.lower_bound)
            {
              m_up = true;
              m_t1_curr_value += 2*m_args.increment;
              m_t2_curr_value += 2*m_args.increment;
              m_s1_curr_value += 2*m_args.increment;
            }
          }

          m_thruster_1.value = m_t1_curr_value;
          m_thruster_2.value = m_t2_curr_value;
          m_servo_1.value    = m_s1_curr_value;

          dispatch(m_thruster_1);
          dispatch(m_thruster_2);
          dispatch(m_servo_1);
        }

        // RNG mode
        else
        {
          srand(time(NULL));
          msg_num = rand() % 3 + 1;

          if(msg_num == 1)
          {
            value = rand() % 101;
            m_thruster_1.value = (((float)value * 0.01)-0.5)*2;  
            dispatch(m_thruster_1);
          }

          if(msg_num == 2)
          {
            value = rand() % 101;
            m_thruster_1.value = (((float)value * 0.01)-0.5)*2;

            value = rand() % 101;
            m_servo_1.value = (((float)value * 0.01)-0.5)*2;

            dispatch(m_thruster_1);
            dispatch(m_servo_1);
          }


          if(msg_num == 2)
          {
            value = rand() % 101;
            m_thruster_1.value = (((float)value * 0.01)-0.5)*2;

            value = rand() % 101;
            m_servo_1.value = (((float)value * 0.01)-0.5)*2;

            value = rand() % 101;
            m_thruster_2.value = (((float)value * 0.01)-0.5)*2;

            dispatch(m_thruster_1);
            dispatch(m_servo_1);
            dispatch(m_thruster_2);
          }
        }  

        return;
      }
    };
  }
}

DUNE_TASK
