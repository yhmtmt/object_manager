// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_object_manager.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_object_manager.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_object_manager.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include "f_object_manager.hpp"
DEFINE_FILTER(f_object_manager)


f_object_manager::f_object_manager(const char * name): f_base(name), m_state(NULL), m_ais_obj(NULL), m_range(10000), m_dtold(180 * SEC)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
  register_fpar("dtold", &m_dtold, "Time the objects alive from their update.");
  register_fpar("range", &m_range, "The object range of interest.");
}

f_object_manager::~f_object_manager()
{
}

bool f_object_manager::init_run()
{
  if (!m_state)
    return false;
  return true;
}

void f_object_manager::destroy_run()
{
}

bool f_object_manager::proc()
{
  double Renu[9];
  double x, y, z;
  float r, p;
  float vox, voy;
  long long t = 0;
  m_state->get_enu_rotation(t, Renu);
  m_state->get_position_ecef(t, x, y, z);
  m_state->get_velocity_vector(t, vox, voy);
  
  if(m_ais_obj){
    // update enu coordinate
    m_ais_obj->update_rel_pos_and_vel(Renu, x, y, z);
    m_ais_obj->remove_old(get_time() - m_dtold);
    m_ais_obj->remove_out(m_range);
    
    m_ais_obj->reset_updates();
    m_ais_obj->calc_tdcpa(t, vox, voy);
  }
  
  return true;
}
