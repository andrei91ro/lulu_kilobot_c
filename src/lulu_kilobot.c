/**
 * @file lulu_kilobot.c
 * @brief Lulu P colony simulator extension that allows controlling a Kilobot robot by using a P colony based controller
 * In this file we implement all methods that allow a complete Kilobot application to be built.
 * @author Andrei G. Florea
 * @author Catalin Buiu
 * @date 2016-03-06
 */

#include "lulu_kilobot.h"
#include "instance.h"
#include "debug_print.h"

#ifdef SIMULATOR
    char* motionNames[] = {"stop", "straight", "left", "right"};
    char* colorNames[] = {"off", "red", "green", "blue"};
#endif

//get *mydata to the coresponding USERDATA structure of the running robot
REGISTER_USERDATA(USERDATA)

void set_motion(motion_t dir_new)
{
	//if the same direction is requested again
	if (dir_new == mydata->current_motion_state)
		//skip request in order to avoid an unneeded engine spinup
		return;
	switch (dir_new)
	{
		case MOTION_STOP:
			set_motors(0, 0);
			break;

		case MOTION_STRAIGHT:
			spinup_motors();
			set_motors(kilo_straight_left, kilo_straight_right);
			break;

		case MOTION_LEFT:
			spinup_motors();
			set_motors(kilo_turn_left, 0);
			break;

		case MOTION_RIGHT:
			spinup_motors();
			set_motors(0, kilo_turn_right);
			break;
	}
	mydata->current_motion_state = dir_new;
}

void procOutputModule(USERDATA *data) {
    #ifdef USING_AGENT_MOTION
        for (uint8_t obj_id = 0; obj_id < data->pcol.agents[AGENT_MOTION].obj.size; obj_id++)
            switch (data->pcol.agents[AGENT_MOTION].obj.items[obj_id]) {
                case OBJECT_ID_M_0: set_motion(MOTION_STOP); break;
                case OBJECT_ID_M_S: set_motion(MOTION_STRAIGHT); break;
                case OBJECT_ID_M_L: set_motion(MOTION_LEFT); break;
                case OBJECT_ID_M_R: set_motion(MOTION_RIGHT);break;
            }
    #endif

    #ifdef USING_AGENT_LED_RGB
        for (uint8_t obj_id = 0; obj_id < data->pcol.agents[AGENT_LED_RGB].obj.size; obj_id++)
            switch (data->pcol.agents[AGENT_LED_RGB].obj.items[obj_id]) {
                case OBJECT_ID_C_0: data->current_led_color = COLOR_OFF; break;
                case OBJECT_ID_C_R: data->current_led_color = COLOR_RED; break;
                case OBJECT_ID_C_G: data->current_led_color = COLOR_GREEN; break;
                case OBJECT_ID_C_B: data->current_led_color = COLOR_BLUE; break;
            }
        set_color(colorValues[data->current_led_color]);
    #endif
}

void loop() {
    //if the previous step was the last one
    if (mydata->sim_result == SIM_STEP_RESULT_NO_MORE_EXECUTABLES) {
        //mark the end of the simulation and exit
        set_color(colorValues[COLOR_GREEN]);
        set_motion(MOTION_STOP);
        printi(("robot_uid %d: SIM_STEP_RESULT_NO_MORE_EXECUTABLES", kilo_uid));
        return;
    } else
    // if the previous step resulted in an error
    if (mydata->sim_result == SIM_STEP_RESULT_ERROR) {
        set_color(colorValues[COLOR_RED]);
        set_motion(MOTION_STOP);
        printi(("robot_uid %d: SIM_STEP_RESULT_ERROR", kilo_uid));
        return;
    }

    //getState(mydata);
    //procInputModule(mydata);
    mydata->sim_result = pcolony_runSimulationStep(&mydata->pcol);
    //transform symbolic objects into effector commands
    procOutputModule(mydata);
}

void setup() {
    //initialize the mydata structure
    mydata->current_led_color = COLOR_OFF;
    mydata->current_motion_state = MOTION_STOP;
    mydata->sim_result = SIM_STEP_RESULT_FINISHED;
    mydata->light = mydata->light_prev = 0;

    lulu_init(&mydata->pcol);
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
char *cb_botinfo(void)
{
  char *p = botinfo_buffer;
  p += sprintf (p, "ID: %d, MOTION: %s, COLOR: %s \n", kilo_uid, motionNames[mydata->current_motion_state], colorNames[mydata->current_led_color]);

  return botinfo_buffer;
}
#endif

int main() {
    kilo_init();
    #ifdef DEBUG
        // initialize serial module (only on Kilobot)
	    debug_init();
    #endif
    //callbacks for Kilombo (resolve to empty functions if building for Kilobot)
    SET_CALLBACK(botinfo, cb_botinfo);

    kilo_start(setup, loop);

    return 0;
}
