#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)

#include "msu.h"
#include "../../module/servo.h"

float selected_filament_nbr=-1;
float absolute_position=0;

float idler_first_filament_pos = 30;
float idler_angle_between_bearing = 26;
float bowdenTubeLength = MSU_BOWDEN_TUBE_LENGTH;

bool idler_engaged = false;

xyze_pos_t position;

float steps_per_mm_correction_factor = 1;
#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  steps_per_mm_correction_factor =  static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#endif
void MSUMP::tool_change(uint8_t index){
  if(selected_filament_nbr>=0){
    #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
      idler_select_filament_nbr(selected_filament_nbr);
    #endif //MSU_DIRECT_DRIVE_SETUP
  }
}

//move idler to specific filament selection, -1 to park the idler
void MSUMP::idler_select_filament_nbr(int index){
  if(index==-1)MOVE_SERVO(MSU_SERVO_IDLER_NBR,270);
  else MOVE_SERVO(MSU_SERVO_IDLER_NBR,MSU_SERVO_OFFSET+(index+1)*MSU_BEARING_ANGLES);
}

#endif