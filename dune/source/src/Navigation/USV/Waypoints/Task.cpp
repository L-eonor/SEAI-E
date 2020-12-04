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

namespace Navigation
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Carlos Pinto
  namespace USV
  {
    namespace Waypoints
    {
      using DUNE_NAMESPACES;

      struct Task: public DUNE::Tasks::Periodic
      {
        struct Arguments
      {
        float test_frequency;
      };

        unsigned int m_iterations;
        Arguments m_args;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Periodic(name, ctx)
        {
          param("Test Frequency", m_args.test_frequency)
          .defaultValue("1")
          .description("Frequency between each test iteration");

          m_iterations = 0;
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.test_frequency))
          {
            setFrequency(m_args.test_frequency);
          }
          IMC::EstimatedState origin_state;
          IMC::DesiredHeading initial_heading_setpoint;
          IMC::DesiredSpeed initial_speed_setpoint;

          initial_heading_setpoint.value = 0;
          initial_speed_setpoint.value = 0;
          origin_state.u = 0;
          origin_state.psi = 0;

          dispatch(initial_heading_setpoint);
          dispatch(initial_speed_setpoint);
          dispatch(origin_state);
          dispatch(origin_state);
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
        task(void)
        {
          spew("\n\n\t---ITERATION %d", m_iterations);
          
          if (0 == m_iterations)
          {
            IMC::DesiredSpeed first_speed_setpoint;
            IMC::DesiredHeading first_heading_setpoint;
            
            first_heading_setpoint.value = Math::c_pi / 2;
            first_speed_setpoint.value = 3.0;
            dispatch (first_heading_setpoint);
            dispatch (first_speed_setpoint);
            spew ("Changed Setpoint to be pi/2 and 3 knots!");
          }
          else if (11 > m_iterations && m_iterations > 0)
          {
            IMC::EstimatedState intermediate_state;
            intermediate_state.psi = (m_iterations) * Math::c_pi/2.0/10.0;
            intermediate_state.u = (m_iterations) * 3.0/10.0;
            spew ("Changed estimated state to %f rad and %f knots.", intermediate_state.psi, intermediate_state.u);
            dispatch(intermediate_state);
          }


          else if (11 == m_iterations)
          {
            IMC::DesiredSpeed first_speed_setpoint;
            IMC::DesiredHeading first_heading_setpoint;
            
            first_heading_setpoint.value = -Math::c_pi/2.0;
            first_speed_setpoint.value = 5.0;
            dispatch (first_heading_setpoint);
            dispatch (first_speed_setpoint);
            spew ("Changed Setpoint to be -pi/2 and 5 knots!");
          }
          else if (21 > m_iterations && m_iterations > 11)
          {
            IMC::EstimatedState intermediate_state;
            intermediate_state.psi = ((m_iterations-10)) * -Math::c_pi/2.0/10.0;
            intermediate_state.u = ((m_iterations-10)) * 5.0/10;
            spew ("Changed estimated state to %f rad and %f knots.", intermediate_state.psi, intermediate_state.u);
            dispatch(intermediate_state);
          }

          else if (40 > m_iterations)
          {
            IMC::EstimatedState intermediate_state;
            intermediate_state.psi = m_iterations/10.0 * Math::c_pi/2;
            intermediate_state.u = m_iterations/10.0;
            spew ("Changed estimated state to %f rad and %f knots.", intermediate_state.psi, intermediate_state.u);
            dispatch(intermediate_state);
          }
          else
          {
            IMC::EstimatedState intermediate_state;
            intermediate_state.u = 5.0;
            intermediate_state.psi = -Math::c_pi/2.0;
            spew ("Changed estimated state to %f rad and %f knots.", intermediate_state.psi, intermediate_state.u);
            dispatch(intermediate_state);
          }
          
          

          m_iterations++;
        }
      };
    }
  }
}

DUNE_TASK
