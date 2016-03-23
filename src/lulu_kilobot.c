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
    char* colorNames[] = {"off", "red", "green", "blue", "white"};
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

void process_message() {
    uint8_t i, smallest_empty_id = MAX_NEIGHBORS;


    uint8_t distance = estimate_distance(&RB_front().dist);
    uint8_t *data = RB_front().msg.data;
    uint8_t id = data[INDEX_MSG_OWNER_UID];

    //in the same loop we check if id appears among the neighbors
    //and also determine (if it exists) the first empty slot in the neighbor list
    for (i = 0; i < mydata->nr_neighbors; i++)
        if (mydata->neighbors[i].uid == id)
            break;
        //// check if this is an empty slot and we haven't found the smallest_empty_id
        //else if (mydata->neighbors[i].uid == 0 && smallest_empty_id == MAX_NEIGHBORS - 1)
            //smallest_empty_id = i;

    // if this neighbor was not found in the list
    if (i >= mydata->nr_neighbors) {
        //add it to the list into an empty slot
        //i = smallest_empty_id;
        //if we need to extend the current list of neighbors
        //if (smallest_empty_id >= mydata->nr_neighbors && smallest_empty_id != MAX_NEIGHBORS)
            mydata->nr_neighbors = (mydata->nr_neighbors < MAX_NEIGHBORS - 1)?mydata->nr_neighbors + 1: MAX_NEIGHBORS - 1;
    }

    mydata->neighbors[i].uid = id;
    mydata->neighbors[i].symbolic_id = id - smallest_robot_uid;
    //if there is no previous recording of the distance to this neighbor
    if (mydata->neighbors[i].distance == mydata->neighbors[i].distance_prev &&
           mydata->neighbors[i].distance == 0)
        //save current distance in both fields
        mydata->neighbors[i].distance = mydata->neighbors[i].distance_prev = distance;
    else {
        //save previous distance
        mydata->neighbors[i].distance_prev = mydata->neighbors[i].distance;
        //save current distance
        mydata->neighbors[i].distance = distance;
    }
    //record the time when this neighbor was processed
    mydata->neighbors[i].timestamp = kilo_ticks;
}

void procInputModule() {
    #ifdef USING_AGENT_MSG_DISTANCE
        for (uint8_t obj_id = 0; obj_id < mydata->pcol.n; obj_id++) {
            if (mydata->pcol.agents[AGENT_MSG_DISTANCE].obj.items[obj_id] == OBJECT_ID_D_ALL) {
                bool dist_big = TRUE;
                for (uint8_t i = 0; i < mydata->nr_neighbors; i++)
                    if (mydata->neighbors[i].distance < PARAM_DISTANCE_THRESHOLD) {
                        dist_big = FALSE;
                        break;
                    }
                if (dist_big || mydata->nr_neighbors == 0)
                    replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_B_ALL);
                else
                    replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_S_ALL);
            }
        }
    #endif
}

void procOutputModule() {
    #ifdef USING_AGENT_MOTION
        for (uint8_t obj_id = 0; obj_id < mydata->pcol.agents[AGENT_MOTION].obj.size; obj_id++)
            switch (mydata->pcol.agents[AGENT_MOTION].obj.items[obj_id]) {
                case OBJECT_ID_M_0: set_motion(MOTION_STOP); break;
                case OBJECT_ID_M_S: set_motion(MOTION_STRAIGHT); break;
                case OBJECT_ID_M_L: set_motion(MOTION_LEFT); break;
                case OBJECT_ID_M_R: set_motion(MOTION_RIGHT);break;
            }
    #endif

    #ifdef USING_AGENT_LED_RGB
        for (uint8_t obj_id = 0; obj_id < mydata->pcol.agents[AGENT_LED_RGB].obj.size; obj_id++)
            switch (mydata->pcol.agents[AGENT_LED_RGB].obj.items[obj_id]) {
                case OBJECT_ID_C_0: mydata->current_led_color = COLOR_OFF; break;
                case OBJECT_ID_C_R: mydata->current_led_color = COLOR_RED; break;
                case OBJECT_ID_C_G: mydata->current_led_color = COLOR_GREEN; break;
                case OBJECT_ID_C_B: mydata->current_led_color = COLOR_BLUE; break;
                case OBJECT_ID_C_W: mydata->current_led_color = COLOR_WHITE; break;
            }
        set_color(colorValues[mydata->current_led_color]);
    #endif
}

message_t* message_tx() {
    return &mydata->msg_tx;
}

void message_rx(message_t* msg, distance_measurement_t* dist) {
    //add one message at the tail of the receive ring buffer
    Received_message_t *newMsg = &RB_back();
    newMsg->msg = *msg;
    newMsg->dist = *dist;
    //now RB_back() will point to an empty slot in the buffer
    RB_pushback();
}

void loop() {
    printi(("\n-----------------------\n LOOP for robot %d\n", kilo_uid));
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

    //process the entire received message buffer
    while (!RB_empty()) {
        process_message();
        RB_popfront();
    }

    //transform sensor input into symbolic objects
    procInputModule();
    mydata->sim_result = pcolony_runSimulationStep(&mydata->pcol);
    //transform symbolic objects into effector commands
    procOutputModule();
}

void setup() {
    //initialize the mydata structure
    mydata->current_led_color = COLOR_OFF;
    mydata->current_motion_state = MOTION_STOP;
    mydata->sim_result = SIM_STEP_RESULT_FINISHED;
    mydata->light = mydata->light_prev = 0;
    mydata->nr_neighbors = 0;

    //initialize message for transmission
    mydata->msg_tx.type = NORMAL;
    mydata->msg_tx.data[INDEX_MSG_OWNER_UID] = kilo_uid; //kilo_uid < 255 so the conversion from 16 to 8 bits should go smoothly
    mydata->msg_tx.crc = message_crc(&mydata->msg_tx);

    //initialize Pcolony
    lulu_init(&mydata->pcol);

    //init neighbors
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        mydata->neighbors[i] = (Neighbor_t) {
            .uid = NO_ID,
            .symbolic_id = NO_ID,
            .distance = 0,
            .distance_prev = 0,
            .timestamp = 0};

    //initialize message receive buffer
    RB_init();
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
char *cb_botinfo(void)
{
  char *p = botinfo_buffer;
  p += sprintf (p, "ID: %d, MOTION: %s, COLOR: %s \n", kilo_uid, motionNames[mydata->current_motion_state], colorNames[mydata->current_led_color]);
  p += sprintf (p, "\n nr_neighbors: %d, neighbor[0].uid: %d neighbor[0].dist: %d", mydata->nr_neighbors, mydata->neighbors[0].uid, mydata->neighbors[0].distance);

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

    //register kilobot callbacks
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

    kilo_start(setup, loop);

    return 0;
}
