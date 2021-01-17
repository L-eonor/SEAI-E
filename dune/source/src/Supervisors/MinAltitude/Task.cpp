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

namespace Supervisors
{
  //! Aborts if navigating in low depth waters.
  //!
  //! Mostly relevant to ASVs, this task reacts to Altitude IMC messages, and,
  //! if navigating in smaller than a parameter-defined threshold altitude,
  //! an Abort is sent.
  //! @author Carlos Pinto
  namespace MinAltitude
  {
    using DUNE_NAMESPACES;


      struct Arguments
      {
        //! Altitude threshold
        fp32_t min_acceptable_altitude;
        //! Altitude Margin that will be used in order to get out of a bad spot
        fp32_t min_altitude_margin;
        //! Value to be used to consider that altitude is back to acceptable levels.
        fp32_t good_altitude_margin;
      };

    struct Task: public DUNE::Tasks::Task
    {
      //! Last depth received (current depth)
      float m_curr_altitude;
      //! Control loops last reference
      uint32_t m_scope_ref;
      //! Abort message to send in case min altitude is breached
      IMC::Abort m_abort;
      //! If the minimum altitude has been surpassed, and we're notw trying to get out of the low altitude area
      bool m_low_altitude;
      //! Task arguments (parameters)
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
          param("Minimum Acceptable Altitude",m_args.min_acceptable_altitude)
          .defaultValue("4.0")
          .description("Minimum value of altitude that the ASV can safely navigate in");

          param("Minimum Altitude Margin",m_args.min_altitude_margin)
          .defaultValue("0.1")
          .description("Extra margin to low altitude, in order to maneuver out of dangerously low altitude waters");

          param("Good Altitude Margin",m_args.good_altitude_margin)
          .defaultValue("0.1")
          .description("Altitude margin to consider that the system is back to a safe altitude");

          // Assume infinite initial depth
          m_curr_altitude = __FLT_MAX__;
          m_scope_ref = 0;
          m_low_altitude = false;
          // Abort "this" system only
          m_abort.setDestination(getSystemId());

          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          bind<IMC::EstimatedState>(this);
          bind<IMC::Abort>(this);
          bind<IMC::ControlLoops>(this);
      }

      void
      onActivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      onDeactivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.min_acceptable_altitude) ||
            paramChanged(m_args.min_altitude_margin)     ||
            paramChanged(m_args.good_altitude_margin))
        {
          // Need to re-evaluate current altitude
          evaluateAltitude();
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


      void
      evaluateAltitude()
      {
        if (!m_low_altitude)
        {
          if (m_curr_altitude < m_args.min_acceptable_altitude)
          {
            err("Aborted mission due to unsafe altitude. Send maneuver in order to navigate out of this area.");
            spew ("The altitude that triggered this error was: %f m", m_curr_altitude);
            dispatch(m_abort, DUNE::Tasks::DispatchFlags::DF_LOOP_BACK);
            m_low_altitude = true;
          }
        }
        else if (m_low_altitude)
        {
          if (m_curr_altitude < m_args.min_acceptable_altitude - m_args.min_altitude_margin)
          {
            err("Aborted mission due to unsafe altitude. Altitude is too low to navigate, retrieve the system.");
            spew ("The altitude that triggered this error was: %f m", m_curr_altitude);
            dispatch(m_abort, DUNE::Tasks::DispatchFlags::DF_LOOP_BACK);
          }
          else if (m_curr_altitude > m_args.min_acceptable_altitude + m_args.good_altitude_margin)
          {
            m_low_altitude = false;
            inf("Altitude is back to acceptable levels.");
            spew ("The altitude that triggered this event was: %f m", m_curr_altitude);
          }
        }
      }

      void
      consume(const IMC::Abort *msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        requestDeactivation();
      }

      void
      consume(const IMC::EstimatedState *msg)
      {
        if (!isActive())
          return;

        m_curr_altitude = msg->alt;

        evaluateAltitude();
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
    };
  }
}


DUNE_TASK
