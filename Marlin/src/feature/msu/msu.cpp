#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)

#include "msu.h"
#include "../../MarlinCore.h"
#include "../../gcode/gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../gcode/parser.h"
#include "../../module/endstops.h"


#if ENABLED(MSU_SERVO_IDLER)
  #include "../../module/servo.h"
#endif

float idlerPosition;

float offsetEndstopTo1 = 3.9;
float spaceBetweenBearings = 3;

float servopos1=20;
float servobearingangle=26;

float absolutePosition;
float storeExtruderPosition; 
float bowdenTubeLength = MSU_BOWDEN_TUBE_SETUP_LENGTH;
float nozzleExtruderGearLength = MSU_NOZZLE_EXTRUDER_GEAR_LENGTH;

int SelectedFilamentNbr = -1;

bool idlerEngaged = true;
bool idlerHomed=false;

//filament loading status, used for error handling
bool changingFilament=false;
bool loadingFilament=false;
bool unloadingFilament=false;
bool homingIdler=false;

xyze_pos_t position;

float steps_per_mm_correction_factor =1;
#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  float msusteps = MSU_EXTRUDER_STEPS_PER_MM;
  float steps;
#endif

void MSUMP::tool_change(uint8_t index)

{
  #if ENABLED (MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  steps = static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
  steps_per_mm_correction_factor =  msusteps /steps;
  #endif
  changingFilament=true;

  #ifdef MSU_DIRECT_DRIVE_SETUP
    if(!idlerEngaged)
  {
    idler_select_filament_nbr(SelectedFilamentNbr);
  }
  #endif//MSU_DIRECT_DRIVE_SETUP
  
  if(!idlerHomed)idler_home();

  //clear the extruder gears
  #ifdef MSU_DIRECT_DRIVE_SETUP
    move_extruder((-nozzleExtruderGearLength-5)*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,10,true);
  #endif

  //disengage idler and clear the extruder with actual extruder
  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    
  //disengage idler if it's engaged  
  if(idlerEngaged)
  {
    idler_select_filament_nbr(-1);
  }
  //clear the extruder gears
  move_extruder(-nozzleExtruderGearLength*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR, 10);

  idler_select_filament_nbr(SelectedFilamentNbr);//get idler back into position to completly extract

  #endif //MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP

  //unload filament until it clears the merger
  move_extruder(-bowdenTubeLength*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,25);

  idler_select_filament_nbr(index);
  SelectedFilamentNbr = index;
  
  //reload the new filament up to the nozzle/extruder gear if running a direct drive setup
  move_extruder(bowdenTubeLength*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,25);

  #ifdef MSU_DIRECT_DRIVE_SETUP
    //put extra pressure to help the extruder gears grab the filament
    move_extruder(1.5*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,10);
    //finish loading with both extruders
    move_extruder(nozzleExtruderGearLength*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,10,true);
  #endif

  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    //put extra pressure to help the extruder gears grab the filament, this is a synched move with both the MSU and the actual extruder
    move_extruder(3*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,10)
    //disengage idler
    idler_select_filament_nbr(-1);
    //finish loading
    move_extruder(nozzleExtruderGearLength*steps_per_mm_correction_factor,MSU_EXTRUDER_ENBR,10);
  #endif //MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
  
  //if direct drive park the idler, may change for direct drive setups to allow for filament "prefeed" with the MSU which would help reduce the strain on the extruder
  #ifdef MSU_DIRECT_DRIVE_SETUP
    idler_select_filament_nbr(-1);
  #endif//MSU_DIRECT_DRIVE_SETUP

  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    idler_select_filament_nbr(-1);
  #endif//MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP

  idlerPosition = absolutePosition;
  changingFilament=false;
  
}


//moves the idler to the specified position, negative value means parking the idler
void MSUMP::idler_select_filament_nbr(int index)
{ //TODO add limits: index can't be higher than the number of filaments
  #if ENABLED(MSU_SERVO_IDLER)
    MOVE_SERVO(MSU_SERVO_IDLER_NBR,servopos1+index*servobearingangle);
  #else
    absolutePosition = offsetEndstopTo1 + index * spaceBetweenBearings;
    //park idler
    if(index<0)absolutePosition=0;

    const float old = current_position.e;
    current_position.e += -(absolutePosition - idlerPosition);
    planner.buffer_line(current_position,  5, MSU_IDLER_ENBR);
    current_position.e = old;
    planner.set_e_position_mm(old);
    planner.synchronize();

    if(index<0)idlerEngaged=false;;
  #endif
}


//homing sequence of the idler. If this is called when using the servo motor it will initiate it

void MSUMP::idler_home()
{
  #if ENABLED(MSU_SERVO_IDLER)
    msu.idler_servo_init();
  #else
    homingIdler = true;
    endstops.enable(true);


    const float old = current_position.e;
    current_position.e += 100;
    planner.buffer_line(current_position,  4, MSU_IDLER_ENBR);
    current_position.e = old;
    planner.set_e_position_mm(old);
    planner.synchronize();
                                               //wait for the move to finish
    endstops.validate_homing_move();
    homingIdler = false;              //homing completed
    idlerPosition = 0;                //new idler position
    endstops.not_homing();
  #endif
  idlerHomed=true;
  
}

#if ENABLED(MSU_SERVO_IDLER)
//servo initiation sequence
void MSUMP::idler_servo_init(){
  idler_select_filament_nbr(0);
}
#endif




//used in the homing process. Used to fix the cold extrusion false trigger when performing idler moves
bool MSUMP::idler_is_homing()
{
  return homingIdler;
}

void MSUMP::edit_MSU_BOWDEN_TUBE_SETUP_length(const float diff){
  bowdenTubeLength+=diff;
}
void MSUMP::move_extruder(float dist, uint8_t extruderNumber,const_feedRate_t speed, bool moveBothExtruders){

  //current_position.e+= dist;
  if (moveBothExtruders)
  {
    const float old = current_position.e;
    current_position.e += dist;
    planner.buffer_line(current_position, speed, MSU_EXTRUDER_ENBR);
    current_position.e = old;
    planner.set_e_position_mm(old);

#if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    const float old = current_position.e;
    current_position.e += dist;
    planner.buffer_line(current_position, speed, MSU_ORIGINAL_EXTRUDER_ENBR);
    current_position.e = old;
    planner.set_e_position_mm(old);
    planner.synchronize();

    #endif
  }

  else
  {

    const float old = current_position.e;
    current_position.e += dist;
    planner.buffer_line(current_position, speed, extruderNumber);
    current_position.e = old;
    planner.set_e_position_mm(old);
    planner.synchronize();
  }
}
void MSUMP::filament_runout(){
  //TODO error handling for filament runout when the MSU is loading/unloading filament
  #if ENABLED(FILAMENT_MOTION_SENSOR)
  if(loadingFilament)error_on_load();
  if(unloadingFilament)error_on_unload();
  #endif
}


void MSUMP::error_on_load(){
  //retract filament
  position.e= -10;
  planner.buffer_line(position,  10, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();
  planner.position.resetExtruder();
  //try loading it again
  position.e= 20;
  planner.buffer_line(position,  10, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();
  planner.position.resetExtruder();

//TODO handle direct drive, add "attempt counter" to call for the user when the printer is unable to fix the load
}

void MSUMP::error_on_unload(){
  //push filament inside the nozzle
  position.e= 10;
  planner.buffer_line(position,  10, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();
  planner.position.resetExtruder();

  gcode.dwell(2000);//fait for filament to "take shape"

  //retract it
  position.e= -20;
  planner.buffer_line(position,  10, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();
  planner.position.resetExtruder();

//TODO handle direct drive, add "attempt counter" to call for the user when the printer is unable to fix the unload
}

const float MSUMP::get_MSU_BOWDEN_TUBE_SETUP_length(){
  return bowdenTubeLength;
}
bool MSUMP::active_filament_change(){
  return changingFilament;
}


#endif
