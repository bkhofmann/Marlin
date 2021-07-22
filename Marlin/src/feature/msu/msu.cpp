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

float idlerPosition; //stores the idler position in mm
float offsetEndstopTo1 = 3.9;//space from the endstop to the first bearing position(Filament 1)
float spaceBetweenBearings = 3;//space in between each bearing
float servopos1=20;//first bearing position
float servobearingangle=27.5;//space between each bearings
float parkedPosition = 0; //this is the parked position. when using the servo it will be the parked position in degree
float absolutePosition;  //used to represent the position in mm where the idler aligns with the correct filament
float storeExtruderPosition; //used to store the extruder position before the tool change so that we are able to reset everything.
float bowdenTubeLength = MSU_BOWDEN_TUBE_SETUP_LENGTH;
float nozzleExtruderGearLength = MSU_NOZZLE_EXTRUDER_GEAR_LENGTH;

int SelectedFilamentNbr = -1; //default to home position until at least one tool change has been performed

bool idlerEngaged = true;//idler engaged or not, this is used in direct drive setup with the MSU disengaging and letting the extruder do everything
bool idlerHomed=false;
bool changingFilament=false;//keeps track of whether the MSU is performing a tool change or not. Will be used to trigger loading failure correction


bool loading=false;
bool unloading=false;


bool homingIdler=false;//homing status used in the homing sequence, but will also be useful in order to disable the bug where the idler won't move if the nozzle is cold(prevent cold extrusion feature)
xyze_pos_t position;//we have to create a fake destination(x,y,z) when doing our MSU moves in order to be able to apply motion limits. We then apply the extruder movement we want to that


#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  double steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM / planner.settings.axis_steps_per_mm[E_AXIS_N(MSU_EXTRUDER_ENBR)];
#else
  double steps_per_mm_correction_factor = 1;
#endif



void MSUMP::tool_change(uint8_t index)
{ changingFilament=true;
  

  #ifdef MSU_DIRECT_DRIVE_SETUP
    if(!idlerEngaged)
  {
    idler_select_filament_nbr(SelectedFilamentNbr);
  }
  #endif//MSU_DIRECT_DRIVE_SETUP
  
  if(!idlerHomed)idler_home();
  
  position= current_position;//get the current position of the nozzle, this will be used so that we don't move the axis when performing any moves at the MSU level
  storeExtruderPosition = planner.position.e;//get the extruder position to be able to revert it once the tool change is done
  apply_motion_limits(position);//apply the motion limits (this is what is used when you create an offset for the bed in the X and Y axis to get proper alignement),
  // if not used the nozzle will move to the wrong position at every tool change
  planner.position.resetExtruder(); //reset the extruder position to 0 to avoid problems with next move
  
  //clear the extruder gears
  #ifdef MSU_DIRECT_DRIVE_SETUP
    planner.position.resetExtruder();
    position.e=  (-nozzleExtruderGearLength-5)*steps_per_mm_correction_factor;//go a bit further just to be sure, it doesn't change anything since the idler is not engaged
    planner.buffer_line(position, 10, MSU_ORIGINAL_EXTRUDER_ENBR);
    planner.position.resetExtruder();
    //also move with the MSU(with the idler putting pressure on the right filament) if the extruder and the MSU are controlled independantly since they have different steps per mm
    position.e=-nozzleExtruderGearLength;
    planner.buffer_line(position, 10, MSU_EXTRUDER_ENBR)//two extruder moves at the same time: needs testing
    planner.synchronize();
  #endif

  //disengage idler and clear the extruder with actual extruder
  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    
  //disengage idler if it's engaged  
  if(idlerEngaged)
  {
    idler_select_filament_nbr(-1);
  }
  //clear the extruder gears
  planner.position.resetExtruder();
  position.e= -nozzleExtruderGearLength*steps_per_mm_correction_factor;
  planner.buffer_line(position, 10, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();

  idler_select_filament_nbr(SelectedFilamentNbr);//get idler back into position to completly extract

  #endif //MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP


  //unload filament until it clears the merger

  position.e= -bowdenTubeLength*steps_per_mm_correction_factor;
  planner.buffer_line(position,  30, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();
  planner.position.resetExtruder();


  idler_select_filament_nbr(index);
  SelectedFilamentNbr = index;
  

  //reload the new filament up to the nozzle/extruder gear if running a direct drive setup
  position.e=bowdenTubeLength*steps_per_mm_correction_factor;
  planner.buffer_line(position, 25, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  planner.synchronize();

  #ifdef MSU_DIRECT_DRIVE_SETUP
    //put extra pressure to help the extruder gears grab the filament
    position.e=1*steps_per_mm_correction_factor;
    planner.buffer_line(position, 10, MSU_EXTRUDER_ENBR)//two extruder moves at the same time: needs testing

    planner.position.resetExtruder();
    position.e= nozzleExtruderGearLength*steps_per_mm_correction_factor;
    planner.buffer_line(position, 10, MSU_ORIGINAL_EXTRUDER_ENBR);
    planner.position.resetExtruder();
    planner.synchronize();
  #endif

  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    //put extra pressure to help the extruder gears grab the filament, this is a synched move with both the MSU and the actual extruder
    position.e=3*steps_per_mm_correction_factor;
    planner.buffer_line(position, 10, MSU_EXTRUDER_ENBR);
    planner.position.resetExtruder();

    //disengage idler
    idler_select_filament_nbr(-1);
    //finish loading
    position.e=nozzleExtruderGearLength*steps_per_mm_correction_factor;
    planner.buffer_line(position, 10, MSU_EXTRUDER_ENBR);
    planner.synchronize();
   
  #endif //MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
  
  //if direct drive park the idler, may change for direct drive setups to allow for filament "prefeed" with the MSU which would help reduce the strain on the extruder
  #ifdef MSU_DIRECT_DRIVE_SETUP
    idler_select_filament_nbr(-1);
  #endif//MSU_DIRECT_DRIVE_SETUP

  #ifdef MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
    idler_select_filament_nbr(-1);
  #endif//MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP

  //reset all the positions to their original state
  planner.synchronize();//wait for all the moves to finish
  planner.position.resetExtruder();
  idlerPosition = absolutePosition;
  planner.position.e = storeExtruderPosition;
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

    position.e=-(absolutePosition - idlerPosition);
    planner.buffer_line(position,  5, MSU_IDLER_ENBR);
    planner.synchronize();
    planner.position.resetExtruder();
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
    position= current_position;
    //apply_motion_limits(position);
    planner.position.resetExtruder();
    position.e= 100;
    planner.buffer_line(position, 4, MSU_IDLER_ENBR); //move towards endstop until it's hit
    planner.synchronize();                                                    //wait for the move to finish
    endstops.validate_homing_move();
    homingIdler = false;              //homing completed
    idlerPosition = 0;                //new idler position
    planner.position.resetExtruder(); //reset the extruder position to 0 to avoid problems with next move
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
void MSUMP::move_msu_extruder(const float diff){
  position.e= -diff;
  planner.buffer_line(position,  20, MSU_EXTRUDER_ENBR);
  planner.position.resetExtruder();
  
}
void MSUMP::filament_runout(){
  //TODO error handling for filament runout when the MSU is loading/unloading filament
  
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
