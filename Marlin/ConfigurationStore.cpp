#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ConfigurationStore.h"
#include "probe.h"
#include "mesh.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.

#define EEPROM_VERSION "V21"

void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homing);
  EEPROM_WRITE_VAR(i,endstop_adj);
  EEPROM_WRITE_VAR(i,delta_radius);
  EEPROM_WRITE_VAR(i,delta_diagonal_rod);
  EEPROM_WRITE_VAR(i,delta_segments_per_second);
  EEPROM_WRITE_VAR(i,delta_tower_angle);
  EEPROM_WRITE_VAR(i,probe_z_offset);
  EEPROM_WRITE_VAR(i,fsr_sensitivity);
  EEPROM_WRITE_VAR(i,delta_calibrated);
  EEPROM_WRITE_VAR(i,mesh_z);
  EEPROM_WRITE_VAR(i,mesh_have);
  EEPROM_WRITE_VAR(i,mesh_active);
  EEPROM_WRITE_VAR(i,mesh_fade_height);
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);

  // Save filament sizes
  
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
void Config_PrintSettings()
{
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[X_AXIS]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[Y_AXIS]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[Z_AXIS]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[E_AXIS]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X", max_feedrate[X_AXIS]);
    SERIAL_ECHOPAIR(" Y", max_feedrate[Y_AXIS]); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[Z_AXIS]); 
    SERIAL_ECHOPAIR(" E", max_feedrate[E_AXIS]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[X_AXIS] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[Y_AXIS] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[Z_AXIS] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[E_AXIS]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homing[X_AXIS] );
    SERIAL_ECHOPAIR(" Y" ,add_homing[Y_AXIS] );
    SERIAL_ECHOPAIR(" Z" ,add_homing[Z_AXIS] );
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop adjustement (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X",endstop_adj[X_AXIS] );
    SERIAL_ECHOPAIR(" Y" ,endstop_adj[Y_AXIS] );
    SERIAL_ECHOPAIR(" Z" ,endstop_adj[Z_AXIS] );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Delta settings: L=delta_diagonal_rod, R=delta_radius, S=delta_segments_per_second");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M665 L",delta_diagonal_rod );
	SERIAL_ECHOPAIR(" R" ,delta_radius );
	SERIAL_ECHOPAIR(" S" ,delta_segments_per_second );
	SERIAL_ECHOPAIR(" A" ,delta_tower_angle[0] );
	SERIAL_ECHOPAIR(" B" ,delta_tower_angle[1] );
	SERIAL_ECHOPAIR(" C" ,delta_tower_angle[2] );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Z probe: M851 Z=nozzle height above the bed at contact, M974 S=FSR sensitivity");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M851 Z",probe_z_offset );
	SERIAL_ECHOPAIR("  M974 S",(unsigned long)fsr_sensitivity );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Bed mesh (G29): M420 S=on/off, Z=fade height");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M420 S",(unsigned long)mesh_active );
	SERIAL_ECHOPAIR(" Z",mesh_fade_height );
	SERIAL_ECHOLN("");
	if (mesh_have) {
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Bed mesh points (index X Y height above Z0, mm):");
		for (uint8_t p = 0; p < MESH_POINTS; p++) {
			float xy[2];
			mesh_point_xy(p, xy);
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR("  ", (unsigned long)p);
			SERIAL_ECHOPAIR(" X", xy[0]);
			SERIAL_ECHOPAIR(" Y", xy[1]);
			SERIAL_ECHOPAIR(" Z", mesh_z[p]);
			SERIAL_ECHOLN("");
		}
	} else {
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Bed mesh: none stored (run G29)");
	}
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki)); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
    SERIAL_ECHOLN(""); 
}

void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homing);
		EEPROM_READ_VAR(i,endstop_adj);
		EEPROM_READ_VAR(i,delta_radius);
		EEPROM_READ_VAR(i,delta_diagonal_rod);
		EEPROM_READ_VAR(i,delta_segments_per_second);
		EEPROM_READ_VAR(i,delta_tower_angle);
		EEPROM_READ_VAR(i,probe_z_offset);
		EEPROM_READ_VAR(i,fsr_sensitivity);
		EEPROM_READ_VAR(i,delta_calibrated);
		EEPROM_READ_VAR(i,mesh_z);
		EEPROM_READ_VAR(i,mesh_have);
		EEPROM_READ_VAR(i,mesh_active);
		EEPROM_READ_VAR(i,mesh_fade_height);
		mesh_changed();
		if (!mesh_have) mesh_active = false;
		recalc_delta_settings(delta_radius, delta_diagonal_rod);  // tower positions follow the loaded radius and rod length
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);

		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    Config_PrintSettings();
}

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homing[X_AXIS] = add_homing[Y_AXIS] = add_homing[Z_AXIS] = 0;
	endstop_adj[X_AXIS] = endstop_adj[Y_AXIS] = endstop_adj[Z_AXIS] = 0;
	delta_radius= DELTA_RADIUS;
	delta_diagonal_rod= DELTA_DIAGONAL_ROD;
	delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
	delta_tower_angle[0] = delta_tower_angle[1] = delta_tower_angle[2] = 0;
	probe_z_offset = 0;
	fsr_sensitivity = FSR_SENSITIVITY;
	delta_calibrated = false;
	for (short m = 0; m < MESH_POINTS; m++) mesh_z[m] = 0;
	mesh_have = mesh_active = false;
	mesh_fade_height = 0;
	mesh_changed();
	recalc_delta_settings(delta_radius, delta_diagonal_rod);
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    


SERIAL_ECHO_START;
SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}
