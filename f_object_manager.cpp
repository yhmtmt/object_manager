// Copyright(c) 2016-2020 Yohei Matsumoto, All right reserved. 

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


f_object_manager::f_object_manager(const char * name): f_base(name),
  m_state(NULL), m_ais_obj(NULL), m_nmea_data(nullptr),
  m_range(10000), m_dtold(180 * SEC)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
  register_fpar("ch_nmea_data", (ch_base**)&m_nmea_data, typeid(ch_nmea_data).name(), "NMEA data channel");
  
  
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
  if(m_nmea_data){
    unsigned int len;
    m_nmea_data->pop(nmea_data_buffer, len);
    if(len != 0){
      auto data =  NMEA0183::GetData(m_nmea_data);
      long long t = data->t();
      const NMEA0183::VDM * vdm = data->payload_as_VDM();
      if(vdm && !vdm->isVDO()){
	switch(vdm->payload_type()){
	case NMEA0183::VDMPayload_PositionReportClassA:{
	  const NMEA0183::PositionReportClassA * pl = 
	    vdm->payload_as_PositionReportClassA();
	  m_ais_obj->push(t, pl->mmsi(),
			  (double) pl->latitude() * (1.0 / (60.0 * 10000.0)),
			  (double) pl->longitude() * (1.0 / (60.0 * 10000.0)),
			  (float)((double) pl->course() * (1.0 / 10.0)),
			  (float)((double) pl->speed() * (1.0 / 10.0)),
			  (float)(pl->heading()));
	}	  
	case NMEA0183::VDMPayload_StandardClassBCSPositionReport:{
	  const NMEA0183::StandardClassBCSPositionReport * pl =
	    vdm->payload_as_StandardClassBCSPositionReport();
	  m_ais_obj->push(t, pl->mmsi(),
			  (double) pl->latitude() * (1.0 / (60.0 * 10000.0)),
			  (double) pl->longitude() * (1.0 / (60.0 * 10000.0)),
			  (float)((double) pl->course() * (1.0 / 10.0)),
			  (float)((double) pl->speed() * (1.0 / 10.0)),
			  (float)(pl->heading()));
	}
	case NMEA0183::VDMPayload_ExtendedClassBCSPositionReport:{
	}
	  const NMEA0183::ExtendedClassBCSPositionReport * pl =
	    vdm->payload_as_ExtendedClassBCSPositionReport();
	  m_ais_obj->push(t, pl->mmsi(),
			  (double) pl->latitude() * (1.0 / (60.0 * 10000.0)),
			  (double) pl->longitude() * (1.0 / (60.0 * 10000.0)),
			  (float)((double) pl->course() * (1.0 / 10.0)),
			  (float)((double) pl->speed() * (1.0 / 10.0)),
			  (float)(pl->heading()));	  
	}
      }
    }
  }
    
    // update enu coordinate
    m_ais_obj->update_rel_pos_and_vel(Renu, x, y, z);
    m_ais_obj->remove_old(get_time() - m_dtold);
    m_ais_obj->remove_out(m_range);
    
    m_ais_obj->reset_updates();
    m_ais_obj->calc_tdcpa(t, vox, voy);
  }
  
  return true;
}
