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

    /* //!Task arguments.
    struct Arguments
    {
      //! PRNG type.
      std::string prng_type;
      //! PRNG seed.
      int prng_seed;
      //! Mean temperature value.
      float mean_value;
      //! Standard deviation of temperature measurements.
      double std_dev;
    }; */
    
    struct Task: public DUNE::Tasks::Periodic
    {
      //! PRNG handle
      //Random::Generator* m_prng;
      //! Task arguments.
      //Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
        //m_prng(NULL)
      {/*
        param("Standard deviation", m_args.std_dev)
        .description("Standard deviation of produced temperature")
        .units(Units::DegreeCelsius)
        .defaultValue("0.1");

        param("PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        param("PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        param("Mean value", m_args.mean_value)
        .description("Mean value of produced temperature")
        .units(Units::DegreeCelsius)
        .defaultValue("25.0"); */
      }

      void
      onEntityReservation(void)
      {
        inf("Starting: %s", resolveEntity(getEntityId()).c_str());
      }

      //! Aquire resources.
      void
      onResourceAcquisition(void)
      {
        /*m_prng = Random::Factory::create(m_args.prng_type,
                                         m_args.prng_seed);*/
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        //Memory::clear(m_prng);
      }


      //! Periodic work.
      void
      task(void)
      {
        IMC::SetThrusterActuation thruster_1, thruster_2;
        IMC::SetServoPosition servo_1;
        uint8_t msg_num;
        int value;
  

        while (!stopping())
        {
          srand(time(NULL));
          msg_num = rand() % 3 + 1;

          if(msg_num == 1)
          {
            thruster_1.id = 1;
            value = rand() % 101;
            thruster_1.value = (((float)value * 0.01)-0.5)*2;  
            thruster_1.setSourceEntity(getEntityId());
            dispatch(thruster_1);
          }

          if(msg_num == 2)
          {
            thruster_1.id = 1;
            value = rand() % 101;
            thruster_1.value = (((float)value * 0.01)-0.5)*2;
            thruster_1.setSourceEntity(getEntityId());

            servo_1.id = 1;
            value = rand() % 101;
            servo_1.value = (((float)value * 0.01)-0.5)*2;
            servo_1.setSourceEntity(getEntityId());

            dispatch(thruster_1);
            dispatch(servo_1);
          }


          if(msg_num == 2)
          {
            thruster_1.id = 1;
            value = rand() % 101;
            thruster_1.value = (((float)value * 0.01)-0.5)*2;
            thruster_1.setSourceEntity(getEntityId());

            servo_1.id = 1;
            value = rand() % 101;
            servo_1.value = (((float)value * 0.01)-0.5)*2;
            servo_1.setSourceEntity(getEntityId());

            thruster_2.id = 2;
            value = rand() % 101;
            thruster_2.value = (((float)value * 0.01)-0.5)*2;
            thruster_2.setSourceEntity(getEntityId());

            dispatch(thruster_1);
            dispatch(servo_1);
            dispatch(thruster_2);
          }
          
          Delay::wait(5.0);
        }

        return;
      }
    };
  }
}

DUNE_TASK
