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
// Author: Eduardo Marques                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <iomanip>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  //! %GPS simulator for DUNE.
  //! %GPS is responsible to gather data from the vehicle simulator
  //! by consuming SimulatedState IMC messages and then add sending
  //! GpsFix messages to the bus.
  //!
  //! This task can also send GroundVelocity and EulerAngles
  //! messages to the bus.
  //!
  //! @author Eduardo Marques.
  namespace LT1000
  {
    using DUNE_NAMESPACES;

    //! GpsFix required validity flags.
    static const uint16_t c_gps_valid = (IMC::GpsFix::GFV_VALID_DATE |
                                         IMC::GpsFix::GFV_VALID_TIME |
                                         IMC::GpsFix::GFV_VALID_POS |
                                         IMC::GpsFix::GFV_VALID_COG |
                                         IMC::GpsFix::GFV_VALID_SOG |
                                         IMC::GpsFix::GFV_VALID_HACC |
                                         IMC::GpsFix::GFV_VALID_VACC |
                                         IMC::GpsFix::GFV_VALID_HDOP |
                                         IMC::GpsFix::GFV_VALID_VDOP);

    //! GroundVelocity required validity flags.
    static const uint8_t c_dvl_valid = (IMC::GroundVelocity::VAL_VEL_X |
                                        IMC::GroundVelocity::VAL_VEL_Y |
                                        IMC::GroundVelocity::VAL_VEL_Z);
    //! %Task arguments.
    struct Arguments
    {
      //! GpsFix report flag.
      bool report_gpsFix;
      //! Yaw report flag.
      bool report_yaw;
      //! Horizontal Dilution of Precision.
      double hdop;
      //! Horizontal Accuracy.
      double hacc;

      //! Initial position (degrees)
      std::vector<double> position;
    };

    //! %GPS simulator task.
    struct Task: public Tasks::Periodic
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! Euler Angles message.
      IMC::EulerAngles m_euler;
      //! Current simulated state.
      IMC::SimulatedState m_sstate;
      //! SimulatedState
      IMC::SimulatedState m_sstate_at_fix;
      //! Origin for simulated state.
      IMC::GpsFix m_origin;
      //! Task arguments.
      Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Periodic(name, ctx)
      {
        // Retrieve configuration parameters.
        param("Report GPS Fix", m_args.report_gpsFix)
        .defaultValue("false")
        .description("Activate output of GPS Fix messages");

        param("Report Yaw", m_args.report_yaw)
        .defaultValue("false")
        .description("Activate output of Euler Angles messages");

        param("HDOP", m_args.hdop)
        .minimumValue("0.0")
        .defaultValue("0.9")
        .description("Horizontal Dilution of Position index");

        param("HACC", m_args.hacc)
        .minimumValue("0.0")
        .defaultValue("2.0")
        .description("Horizontal Accuracy index");

        param("Initial Position", m_args.position)
        .units(Units::Degree)
        .size(2)
        .description("Initial position of the vehicle");

        m_fix.clear();
        m_euler.clear();

        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_WAIT_GPS_FIX);

        m_fix.validity = c_gps_valid;
      }

      void
      onUpdateParameters(void)
      {
        m_fix.lat = Math::Angles::radians(m_args.position[0]);
        m_fix.lon = Math::Angles::radians(m_args.position[1]);
        m_fix.type = IMC::GpsFix::GFT_MANUAL_INPUT;
        m_fix.validity = 0xffff;
      }

      void
      onResourceInitialization(void)
      {
        // Dispatching local origin.
        dispatch(m_fix);

        inf("O QUE É QUE ACONTECE QUANDO INICIALIZA?");
        inf("m_fix.lat:  %f", m_fix.lat);
        inf("m_fix.lon:  %f", m_fix.lon);
        inf("m_fix.type: %d", m_fix.type);
        inf("m_fix.val:  %d", m_fix.validity);


        m_sstate.u= 2;
        m_sstate.v= 1;
        m_sstate.psi= 1.27;
        m_sstate.x= 10;
        m_sstate.y= 100;
        m_sstate.z=0;
      }


      //! Report invalid fix.
      void
      reportInvalidFix(void)
      {
        trace("reporting invalid fix");

        m_fix.validity = 0;
        m_fix.sog = 0.0;
        m_fix.cog = 0.0;
        m_fix.hdop += 1.0 / getFrequency();
        m_fix.hacc += 1.0 / getFrequency();
        m_fix.utc_time = ((uint32_t)Clock::getSinceEpoch()) % 86400;
        dispatch(m_fix);
      }

      void
      task(void)
      {

        double now = Clock::getSinceEpoch();
        inf("NOW:  %lf", now);

        // Report GpsFix.
        m_fix.sog = std::sqrt(std::pow(m_sstate.u, 2) + std::pow(m_sstate.v, 2));
        m_fix.cog = m_sstate.psi;
        m_fix.validity = c_gps_valid;
        m_fix.hdop = m_args.hdop;
        m_fix.hacc = m_args.hacc;

        // WGS84 coordinates.
        m_fix.lat = m_fix.lat;
        m_fix.lon = m_fix.lon;
        m_fix.height = m_fix.height;
        WGS84::displace(m_sstate.x, m_sstate.y, m_sstate.z, &m_fix.lat, &m_fix.lon, &m_fix.height);
        m_fix.utc_time = ((uint32_t)now) % 86400;

        trace("fix: %0.6f %0.6f | yaw %0.1f",
              Angles::degrees(m_fix.lat), Angles::degrees(m_fix.lon),
              Angles::degrees(m_euler.psi));

        m_fix.setTimeStamp(now);
        dispatch(m_fix, DF_KEEP_TIME);


        // Report Heading.
        if (m_args.report_yaw)
        {
          m_euler.psi = m_sstate.psi;
          m_euler.setTimeStamp(now);
          dispatch(m_euler, DF_KEEP_TIME);
        }


         inf("m_euler.psi     -> %f",  m_euler.psi);
         inf("m_fix.sog       -> %lf", m_fix.sog );
         inf("m_fix.cog       -> %lf", m_fix.cog);
         inf("m_fix.validity  -> %d",  m_fix.validity);
         inf("m_fix.lat       -> %lf", m_fix.lat);
         inf("m_fix.lon       -> %lf", m_fix.lon);
         inf("m_fix.height    -> %lf", m_fix.height);
         inf("*****************************************");


      }
    };
  }
}

DUNE_TASK
