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

namespace Simulators
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Carlos Pinto
  namespace USV
  {
    namespace ManeuverAwesomeSender
    {
      using DUNE_NAMESPACES;

      struct Task: public DUNE::Tasks::Periodic
      {

        IMC::Goto m_maneuver_to_send;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Periodic(name, ctx)
        {
          setFrequency(1);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          m_maneuver_to_send.lat = 40;
          m_maneuver_to_send.lon = 8;
          war("MANEUVER SENT");
          dispatch(m_maneuver_to_send);

          IMC::FollowPath another_maneuver;

          MessageList<PathPoint> path_list;
          PathPoint *point = new PathPoint();
          point->x = 0;
          point->y = 0;
          path_list.push_back(point);
          point = new PathPoint();
          point->x = 1;
          point->y = 1;
          path_list.push_back(point);
          point = new PathPoint();
          point->x = 2;
          point->y = 2;
          path_list.push_back(point);
          another_maneuver.points = path_list;

          dispatch (another_maneuver);
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
        task(void)
        {
          war("I'm alive");
        }
      };
    }
  }
}

DUNE_TASK
