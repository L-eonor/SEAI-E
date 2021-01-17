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
#include <cstring>
#include <cstddef>
#include <string>
// DUNE headers.
#include <DUNE/DUNE.hpp>

#define BUFFER_MAX 256

namespace Actuators
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Francisco
  namespace UART_Comm
  {
    //! timeout difference between task periods, in which a problem with uart is considered to have happened
    static const double c_timeout_tolerance = 1.0;
    //! number of serial port messages after which the serial port gets flushed
    static const unsigned int c_msg_flush_count = 8;


    using DUNE_NAMESPACES;

    struct Arguments
    {
      std::string m_trg_prod;
      // Serial port device.
      std::string uart_dev;
      // Serial port baud rate.
      unsigned uart_baud;
      // frequency for sending actuator messages
      float message_frequency;
      // upper bound for actuation values
      int value_upper_bound;
      // lower bound for actuation values
      int value_lower_bound;
    };

    struct Task: public DUNE::Tasks::Periodic
    {

      // Parameters.
      //! Serial Port object for Arduino interfacing
      SerialPort* m_uart;
      //! Boolean that stores if a successful serial port connection has been established
      bool m_uart_is_initialized;
      //! Task Arguments.
      Arguments m_args;
      //! Thruster Port Command
      char m_cmd_thruster_1[65]={};
      //! Thruster Starboard Command
      char m_cmd_thruster_2[65]={};
      //! Rudder Command
      char m_cmd_rudder_1[65]={};
      //! Rudder Command
      char m_cmd_rudder_2[65]={};
      //! Serial Port message counter
      unsigned int m_msg_counter;
      //! previously stored values of the servo and the thrusters
      int m_s1_prev = 0, m_s2_prev = 0, m_t1_prev = 0, m_t2_prev = 0;
      //! previously sent values of the servo and the thrusters
      int m_s1_sent_prev = 0, m_s2_sent_prev = 0, m_t1_sent_prev = 0, m_t2_sent_prev = 0;
      //! Delta between task() calls. If delta is too big, it means task is getting stuck which might indicate a filled buffer in Serial Port
      DUNE::Time::Delta m_task_delta;

      Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Periodic(name, ctx), m_uart(NULL)
      {
        param("Target Producer", m_args.m_trg_prod)
        .description("Target producer to read from")
        .defaultValue("Producer");

        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyACM0")
        .description("Serial port device (used to communicate with the actuator)");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("Message Frequency", m_args.message_frequency)
        .defaultValue("1.0")
        .description("Frequency at which we send Thruster and Rudder Control Messages to Arduino");

        param("Actuation Lower Bound", m_args.value_lower_bound)
        .defaultValue("1000")
        .description("Received actuation values will be trimmed to this bound");

        param("Actuation Upper Bound", m_args.value_upper_bound)
        .defaultValue("2000")
        .description("Received actuation values will be trimmed to this bound");


        createCommand("m",1500);
        createCommand("M",1500);
        createCommand("l",1500);
        createCommand("L",1500);
        m_s1_sent_prev = 1500;
        m_s2_sent_prev = 1500;
        m_t1_sent_prev = 1500;
        m_t2_sent_prev = 1500;
        m_msg_counter = 0;
        m_uart_is_initialized = false;


        bind<IMC::SetThrusterActuation>(this);
        bind<IMC::SetServoPosition>(this);
      }

      void
      onEntityResolution(void)
      {

      }

      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.message_frequency))
        {
          setFrequency(m_args.message_frequency);
        }
      }

      void
      onResourceAcquisition(void)
      {
        try
        {
          m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
          m_uart_is_initialized = true;
        }
        catch(const std::exception& e)
        {
          err ("%s",e.what());
        }
      }

      void
      onResourceRelease(void)
      {
        Memory::clear(m_uart);
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
        int value;
        value = int((msg->value)*500.0 + 1500.0);
        value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);

        if((msg->id) == 1)
        {
          if(value != m_t1_prev)
          {
            // "m" is the motor 1 identifier in the Arduino Sketch
            createCommand("m", value);

            m_t1_prev = value;
          }
        }

        if((msg->id) == 0)
        {
          if (value != m_t2_prev)
          {
            // "M" is the motor 2 identifier in the Arduino Sketch
            createCommand("M", value);

            m_t2_prev = value;
          }

        }
      }

      void
      consume(const IMC::SetServoPosition* msg)
      {
        int value;
        value = int((msg->value)*500.0 + 1500.0);
        value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);

        if (value != m_s1_prev)
        {
          // "l" is the rudder identifier in the Arduino Sketch
          createCommand("l", value);
          createCommand("L", value);

          m_s1_prev = value;
        }
      }

      void
      sendCommand(const char* cmd)
      {
        if (serialPortIsInvalid())
          return;

        m_uart->writeString(cmd);

        if (++m_msg_counter >= c_msg_flush_count)
        {
            m_uart->flush();
            m_msg_counter = 0;
        }
      }

      void createCommand(const std::string& cmd_type, fp32_t val)
      {
        std::stringstream ss;
	      ss << cmd_type << val << "*";
        std::string str = ss.str();

        if (cmd_type[0] == 'M')
              {strcpy(m_cmd_thruster_2, str.c_str());}
        else if (cmd_type[0] == 'm')
              {strcpy(m_cmd_thruster_1, str.c_str());}
        else if (cmd_type[0] == 'l')
              {strcpy(m_cmd_rudder_1,     str.c_str());}
        else if (cmd_type[0] == 'L')
              {strcpy(m_cmd_rudder_2,     str.c_str());}
      }

      bool
      serialPortIsInvalid()
      {
        if (!m_uart_is_initialized)
        {
          try
          {
            m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
            m_uart_is_initialized = true;
          }
          catch(const std::exception& e)
          {
            err ("%s",e.what());
            return true;
          }
        }

        return false;
      }

      void
      task(void)
      {
        if (m_task_delta.getDelta() > ((1.0/(double)m_args.message_frequency) + c_timeout_tolerance))
        {
          // If task took too long to re-run, probably uart buffer is overfilled, so we flush it (both RX and TX).
          m_uart->flush();
        }
        consumeMessages();

        if (m_t1_prev != m_t1_sent_prev)
        {
          spew ("Thruster 1 value updated.");
          sendCommand(m_cmd_thruster_1);
          m_t1_sent_prev = m_t1_prev;
        }
        if (m_t2_prev != m_t2_sent_prev)
        {
          spew ("Thruster 2 value updated.");
          sendCommand(m_cmd_thruster_2);
          m_t2_sent_prev = m_t2_prev;
        }
        if (m_s1_prev != m_s1_sent_prev)
        {
          spew ("Rudder value updated.");
          sendCommand(m_cmd_rudder_1);
          sendCommand(m_cmd_rudder_2);
          m_s1_sent_prev = m_s1_prev;
          m_s2_sent_prev = m_s2_prev;
        }
      }

    };
  }
}
DUNE_TASK