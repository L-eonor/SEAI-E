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

      SerialPort* m_uart;
      // Task Arguments.
      Arguments m_args;
      // Thruster Port Command
      char m_cmd_thruster_1[10];
      // Thruster Starboard Command
      char m_cmd_thruster_2[10];
      // Rudder Command
      char m_cmd_rudder[10];
      // Serial Port buffer.
      //uint8_t m_bfr[BUFFER_MAX];


      //previous values of the servo and the thrusters
      fp32_t s1_prev = 0, t1_prev = 0, t2_prev = 0;
      int value;

      void sendCommand(const char* cmd)
      {

        size_t size_cmd = sizeof(cmd);

        //inf("Size: %d", size_cmd);

        m_uart->write(cmd, size_cmd);
       
        //trace("OUT | %s | %u", sanitize(cmd).c_str(), (unsigned)cmd.size());
        // Check for command success.
        /*if(m_uart->hasNewData(1.0) == IOMultiplexing::PRES_OK)
          {
          int retval = m_uart->read(m_bfr, sizeof(bfr));
          debug("%i", retval);
          }
          else
          {
          debug("no response!");
          }*/
      }

      void createCommand(const std::string& cmd_type, fp32_t val)
      {
        std::stringstream ss;

        ss << cmd_type << val << "*\n";

        std::string str = ss.str();

        //inf("Message: %f\n", val);

        //const char *cmd = str.c_str();

        switch (cmd_type)
        {
          case "M":
            m_cmd_thruster_2 = str.c_str();
            break;
          
          case "m":
            m_cmd_thruster_1 = str.c_str();
            break;
          
          case "l":
            m_cmd_rudder = str.c_str();
            break;
          
          default:
            break;
        }
      
        //sendCommand(cmd);
      }

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL)
      {
        param("Target Producer", m_args.m_trg_prod)
        .description("Target producer to read from")
        .defaultValue("Producer");

        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyACM0")
        .description("Serial port device (used to communicate with the actuator)");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("112500")
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


        std::stringstream ss;
        ss << "m" << "1500" << "*\n";
        std::string str = ss.str();
        m_cmd_thruster_1 = str.c_str();
        ss << "M" << "1500" << "*\n";
        std::string str = ss.str();
        m_cmd_thruster_2 = str.c_str();
        ss << "l" << "1500" << "*\n";
        std::string str = ss.str();
        m_cmd_rudder = ss.str();


        bind<IMC::SetThrusterActuation>(this);
        bind<IMC::SetServoPosition>(this);
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
        m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud,
                                SerialPort::SP_PARITY_NONE,
                                SerialPort::SP_STOPBITS_1,
                                SerialPort::SP_DATABITS_8);
      }

      void
      onResourceRelease(void)
      {
        Memory::clear(m_uart);
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
	      //if (m_trg_prod == msg.get(SourceEntity))
	      if (m_args.m_trg_prod == resolveEntity(msg->getSourceEntity()))
	      {
          value = int((msg->value)*500.0 + 1500.0);
          value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);
          inf("Truster %d's value is %f ", msg->id, value);

          if((msg->id) == 1)
          {
            if(value != t1_prev)
            {
              // "m" is the motor 1 identifier in the Arduino Sketch
              createCommand("m", value);

              inf("Value is %d ", value);

              t1_prev = value;
            }
          }

          if((msg->id) == 0)
          {
            if (value != t2_prev)
            {
              // "M" is the motor 2 identifier in the Arduino Sketch
              createCommand("M", value);

              inf("Value is %d ", value);

              t2_prev = value;
            }

          }
        }
      }


      void
      consume(const IMC::SetServoPosition* msg)
      {
	      //if (m_trg_prod == msg.get(SourceEntity))
	      if (m_args.m_trg_prod == resolveEntity(msg->getSourceEntity()))
	      {
          value = int((msg->value)*500.0 + 1500.0);
          value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);
          inf("Servo's value is %f ", value);

          if (value != s1_prev)
          {
            // "l" is the rudder identifier in the Arduino Sketch
            createCommand("l", value);

            s1_prev = value;
          }
        }
      }
      
      void
      task(void)
      {
        sendCommand(m_cmd_thruster_1);
        sendCommand(m_cmd_thruster_2);
        sendCommand(m_cmd_rudder);
      }
    };
  }
}
DUNE_TASK
