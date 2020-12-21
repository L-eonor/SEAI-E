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
      char m_cmd_thruster_1[65]={};
      // Thruster Starboard Command
      char m_cmd_thruster_2[65]={};
      // Rudder Command
      char m_cmd_rudder[65]={};
      // Serial Port buffer.
      //uint8_t m_bfr[BUFFER_MAX];

      //previously stored values of the servo and the thrusters
      int m_s1_prev = 0, m_t1_prev = 0, m_t2_prev = 0;
      //previously sent values of the servo and the thrusters
      int m_s1_sent_prev = 0, m_t1_sent_prev = 0, m_t2_sent_prev = 0;

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


        createCommand("m",1500);
        createCommand("M",1500);
        createCommand("l",1500);
        m_s1_sent_prev = 1500;
        m_t1_sent_prev = 1500;
        m_t2_sent_prev = 1500;


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
        m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
	      //if (m_trg_prod == msg.get(SourceEntity))
        int value;
	      if (m_args.m_trg_prod == resolveEntity(msg->getSourceEntity()))
	      {
          value = int((msg->value)*500.0 + 1500.0);
          value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);

          if((msg->id) == 1)
          {
            if(value != m_t1_prev)
            {
              // "m" is the motor 1 identifier in the Arduino Sketch
              createCommand("m", value);

              inf("Truster %d's value is %d ", msg->id, value);

              m_t1_prev = value;
            }
          }

          if((msg->id) == 0)
          {
            if (value != m_t2_prev)
            {
              // "M" is the motor 2 identifier in the Arduino Sketch
              createCommand("M", value);

              inf("Truster %d's value is %d ", msg->id, value);

              m_t2_prev = value;
            }

          }
        }
      }

      void
      consume(const IMC::SetServoPosition* msg)
      {
	      //if (m_trg_prod == msg.get(SourceEntity))
        int value;
	      if (m_args.m_trg_prod == resolveEntity(msg->getSourceEntity()))
	      {
          value = int((msg->value)*500.0 + 1500.0);
          value =  DUNE::Math::trimValue(value, m_args.value_lower_bound, m_args.value_upper_bound);

          if (value != m_s1_prev)
          {
          inf("Servo's value is %d ", value);
            // "l" is the rudder identifier in the Arduino Sketch
            createCommand("l", value);

            m_s1_prev = value;
          }
        }
      }

      void sendCommand(const char* cmd)
      {

        size_t size_cmd = strlen(cmd);

	//war("SSS%ssss\n", cmd);
        //inf("Size: %d", size_cmd);

        m_uart->writeString(cmd);

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

        //const char *cmd = str.c_str();

	     if (cmd_type[0] == 'M')
            {strcpy(m_cmd_thruster_2, str.c_str());}
       else if (cmd_type[0] == 'm')
            {strcpy(m_cmd_thruster_1, str.c_str());}
	     else if (cmd_type[0] == 'l')
            {strcpy(m_cmd_rudder,     str.c_str());}
      }

      void
      task(void)
      {
        war("Checking values...\n%s != %d\n%s != %d\n%s != %d", m_cmd_thruster_1, m_t1_prev, m_cmd_thruster_2, m_t2_prev, m_cmd_rudder, m_s1_prev);
        if (m_t1_prev != m_t1_sent_prev)
        {
          war ("Thruster 1 value updated.");
          sendCommand(m_cmd_thruster_1);
          m_t1_sent_prev = m_t1_prev;
        }
        if (m_t2_prev != m_t2_sent_prev)
        {
          war ("Thruster 2 value updated.");
          sendCommand(m_cmd_thruster_2);
          m_t2_sent_prev = m_t2_prev;
        }
        if (m_s1_prev != m_s1_sent_prev)
        {
          war ("Rudder value updated.");
          sendCommand(m_cmd_rudder);
          m_s1_sent_prev = m_s1_prev;
        }
      }

      /*void
      onMain(void)
      {
        while (!stopping())
        {
          //waitForMessages(1.0);

        }
      }*/

    };
  }
}
DUNE_TASK
