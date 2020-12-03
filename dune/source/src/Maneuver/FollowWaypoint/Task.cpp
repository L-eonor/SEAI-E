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
#include <math.h>

namespace Maneuver
{
  //! Cool task
  //!
  //! It does things.
  //! @author Carlos Pinto
  namespace FollowWaypoint
  {
    using DUNE_NAMESPACES;

      static const uint16_t TYPE_FOLLOWPATH = 457;

      struct Arguments
      {
        float horizontal_tolerance;
        float default_speed;
        std::string vehicle_type;
        std::string default_speed_units;
      };

      struct Waypoint
      {
        float x;
        float y;
      };

    struct Task: public DUNE::Maneuvers::Maneuver
    {
      Arguments m_args;
      std::vector<Waypoint> m_wpts;
      uint16_t m_curr = 0;
      IMC::DesiredPath m_path;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx)
      {
        param("Horizontal Tolerance", m_args.horizontal_tolerance)
        .defaultValue("7.5")
        .units(Units::Meter)
        .description("Tolerance in XY in which the Waypoint is considered to be achieved");

        param("Default Speed", m_args.default_speed)
        .defaultValue("1.8")
        .units(Units::MeterPerSecond)
        .description("Default speed, when no specific speed is specified");

        bindToManeuver<Task, IMC::FollowPath>();
        //bindToManeuver<Task, IMC::Goto>();
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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
      consume (const IMC::Maneuver *maneuver)
      {
        spew ("Maneuver received: ID - %d", maneuver->getId());
         if (maneuver->getId() == TYPE_FOLLOWPATH)
         {
           spew("Maneuver was identified as type Follow Path");
           onStart(static_cast<const IMC::FollowPath*>(maneuver));
         }
      }

      void 
      consume (IMC::EstimatedState *state)
      {
        double *lat, *lon;
        double x, y;
        x = state->lat;
        y = state->lon;
        spew ("Received new Vehicle State:\n\tlat: %lf\n\tlon: %lf", x, y);
        Coordinates::WGS84::toECEF((double)state->lat, (double)state->lon, 0.0, &x, &y, (double*)NULL);
        auto x_dist = m_wpts[m_curr].x-x;
        auto y_dist = m_wpts[m_curr].y-y;
        auto distance_to_target = std::sqrt(x_dist * x_dist + y_dist * y_dist);
        spew ("Coordinates translated to ECEF format.\nCurrent:\n\tx: %lf\n\ty: %lf\nDesired:\n\tx: %lf\n\ty: %lf\nTherefore --- Distance to Target: %lf",x, y, m_wpts[m_curr].x, m_wpts[m_curr].y, distance_to_target);

        if (distance_to_target < m_args.horizontal_tolerance)
        {
          spew ("Waypoint %d Reached! Moving to next waypoint...", m_curr);
          m_curr++;
          goToNextPoint();
        }

        // Temporary Implementation using heading+speed rather than sending DesiredPath directly
        IMC::DesiredHeading dsrd_heading;
        dsrd_heading.value = atan2(y_dist, x_dist);
        IMC:DesiredSpeed dsrd_speed;
        dsrd_speed.speed_units = Units::MeterPerSecond;

        if (m_path.speed && m_path.speed_units == Units::MeterPerSecond)
        {
          dsrd_speed.value = m_path.speed;
        }
        else if (m_path.speed && m_path.speed_units == Units::Knot)
        {
          dsrd_speed.value = 0.5144*m_path.speed;
        }
        else
        {
          dsrd_speed.value = m_args.default_speed;
        }

        dispatch(dsrd_speed);
        dispatch(dsrd_heading);
        
      }

      void 
      consume (IMC::Brake *brake)
      {

      }

      inline void 
      goToNextPoint(void)// To do: Por o start_lat e start_lon?
      {
          Waypoint &w = m_wpts[m_curr];
          m_path.end_lat = w.x;
          m_path.end_lon = w.y;
          dispatch (m_path);
      }

      void
      reset(void)
      {
        m_curr = 0;
        m_wpts.clear();
      }

      void
      onStart(const IMC::FollowPath *maneuver)
      {
          reset(); // clears all waypoints

          IMC::MessageList<IMC::PathPoint>::const_iterator itr;

          uint8_t point_number = 1;
          spew ("Origin lattittude,longitude = (%lf,%lf)", maneuver->lat, maneuver->lon);

          for (itr = maneuver->points.begin(); itr != maneuver->points.end(); itr++)
          {// for each PathPoint:
            spew ("point number %d: \nlat: %f\nlon: %f\n\n",point_number++, (*itr)->x, (*itr)->y);
            if ((*itr)==NULL)
              continue;
            

            Waypoint w;
            // note that w.x and w.y will not hold lat and lon, this is just an intermediate step
            w.x = maneuver->lat;
            w.y = maneuver->lon;

            // calculates the offset, in meters, of the given lat and lon to the 
            // maneuver's lat and lon reference
            WGS84::displace ((*itr)->x, (*itr)->y, &w.x, &w.y);
            
            // puts waypoint, as x and y displacement, in waypoint vector
            m_wpts.push_back(w);
          }

          goToNextPoint();
      }
    };
  }
}

DUNE_TASK
