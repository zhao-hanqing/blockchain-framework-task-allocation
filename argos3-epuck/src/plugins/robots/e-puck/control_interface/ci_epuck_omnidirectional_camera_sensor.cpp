/**
 * @file <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.cpp
 *
 * @brief This file provides the definition of the e-puck
 * omnidirectional camera sensor.
 *
 *  * @author Lorenzo Garattoni - <lgaratto@ulb.ac.be>
    * @author Gianpiero Francesca <gianpiero.francesca@ulb.ac.be>
 */
#include "ci_epuck_omnidirectional_camera_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaEnableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_EPuckOmnidirectionalCameraSensor>(pt_lua_state, "colored_blob_omnidirectional_camera")->Enable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaDisableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_EPuckOmnidirectionalCameraSensor>(pt_lua_state, "colored_blob_omnidirectional_camera")->Disable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuckOmnidirectionalCameraSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "colored_blob_omnidirectional_camera");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "enable", &LuaEnableCamera);
      CLuaUtility::AddToTable(pt_lua_state, "disable", &LuaDisableCamera);
      for(size_t i = 0; i < m_sReadings.BlobList.size(); ++i) {
         SBlob& sBlob = *(m_sReadings.BlobList[i]);
         CLuaUtility::StartTable(pt_lua_state, i+1);
         CLuaUtility::AddToTable(pt_lua_state, "distance", sBlob.Distance);
         CLuaUtility::AddToTable(pt_lua_state, "angle", sBlob.Angle);
         CLuaUtility::AddToTable(pt_lua_state, "color", sBlob.Color);
         CLuaUtility::EndTable(pt_lua_state);
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuckOmnidirectionalCameraSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "colored_blob_omnidirectional_camera");
      /* Save the number of elements in the blob list */
      size_t unLastBlobNum = lua_rawlen(pt_lua_state, -1);
      /* Overwrite the table with the new messages */
      for(size_t i = 0; i < m_sReadings.BlobList.size(); ++i) {
         SBlob& sBlob = *(m_sReadings.BlobList[i]);
         CLuaUtility::StartTable(pt_lua_state, i+1);
         CLuaUtility::AddToTable(pt_lua_state, "distance", sBlob.Distance);
         CLuaUtility::AddToTable(pt_lua_state, "angle", sBlob.Angle);
         CLuaUtility::AddToTable(pt_lua_state, "color", sBlob.Color);
         CLuaUtility::EndTable(pt_lua_state);
      }
      /* Are the new messages less than the old ones? */
      if(m_sReadings.BlobList.size() < unLastBlobNum) {
         /* Yes, set to nil all the extra entries */
         for(size_t i = m_sReadings.BlobList.size()+1; i <= unLastBlobNum; ++i) {
            lua_pushnumber(pt_lua_state,  i);
            lua_pushnil   (pt_lua_state    );
            lua_settable  (pt_lua_state, -3);
         }
      }
      lua_pop(pt_lua_state, 1);
   }
   bool CCI_EPuckOmnidirectionalCameraSensor::Enabled() const
   {
       return m_bEnabled;
   }
   

#endif
   
   /****************************************/
   /****************************************/
   
}
