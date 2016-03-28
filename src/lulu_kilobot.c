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

void forget_neighbors() {
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        // if the deadline for forgeting this neighbor has passed
        if (kilo_ticks >= mydata->neighbors[i].timexp_forget && mydata->neighbors[i].timexp_forget > 0) {
            //decrease the number of neighbors
            mydata->nr_neighbors = (mydata->nr_neighbors > 0)? mydata->nr_neighbors - 1: 0;
            //re-initialize this neighbor
            mydata->neighbors[i].uid = NO_ID;
            mydata->neighbors[i].symbolic_id = NO_ID;
            mydata->neighbors[i].distance = 0;
            mydata->neighbors[i].distance_prev = 0;
            mydata->neighbors[i].timexp_forget = 0;
        }
}

void process_message() {
    uint8_t i;


    uint8_t distance = estimate_distance(&RB_front().dist);
    uint8_t *data = RB_front().msg.data;
    uint8_t id = data[INDEX_MSG_OWNER_UID];

    //search for the robot uid in the current neighbor list
    for (i = 0; i < MAX_NEIGHBORS; i++)
        if (mydata->neighbors[i].uid == id)
            break;

    //if the sender of this message was not found in the current list of neighbors
    if (i == MAX_NEIGHBORS)
        //check for an empty slot
        for (i = 0; i < MAX_NEIGHBORS; i++)
            if (mydata->neighbors[i].uid == NO_ID) {
                //we will remember a new neighbor
                mydata->nr_neighbors++;
                break;
            }

    //if there was no empty slot
    if (i == MAX_NEIGHBORS) {
        printw(("kilo_uid: %d - no slot for KB%d", kilo_uid, id));
        return;
    }

    //if we reach this step then i is a valid slot
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

    //set the moment in the future when the robot will forget about this neighbor
    mydata->neighbors[i].timexp_forget = kilo_ticks + FORGET_NEIGHBOR_INTERVAL;
}

void procInputModule() {
    uint8_t i; //used for iterating through the neighbor list

    #ifdef USING_AGENT_MSG_DISTANCE
        for (uint8_t obj_id = 0; obj_id < mydata->pcol.n; obj_id++) {
            #ifdef USING_OBJECT_D_ALL
                if (mydata->pcol.agents[AGENT_MSG_DISTANCE].obj.items[obj_id] == OBJECT_ID_D_ALL) {
                    bool dist_big = TRUE;
                    for (i = 0; i < MAX_NEIGHBORS; i++)
                        if (mydata->neighbors[i].distance < PARAM_DISTANCE_THRESHOLD && mydata->neighbors[i].uid != NO_ID) {
                            dist_big = FALSE;
                            break;
                        }
                    if (dist_big || mydata->nr_neighbors == 0)
                        replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_B_ALL);
                    else
                        replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_S_ALL);
                }
            #endif
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
    //if the previous step was the last one
    if (mydata->sim_result == SIM_STEP_RESULT_NO_MORE_EXECUTABLES) {
        //mark the end of the simulation and exit
        set_color(RGB(0, 2, 0));
        set_motion(MOTION_STOP);
        printi(("kilo_uid %d: NO_MORE_EXEC", kilo_uid));
        return;
    } else
    // if the previous step resulted in an error
    if (mydata->sim_result == SIM_STEP_RESULT_ERROR) {
        set_color(RGB(2, 0, 0));
        set_motion(MOTION_STOP);
        //we don't print any error message here because we wouldn't be able to read the LULU error because this continuous print
        return;
    }

    //remove old neighbors that haven't sent any recent message
    forget_neighbors();

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

    #ifdef NEEDING_WILDCARD_EXPANSION
        expand_pcolony(&mydata->pcol, kilo_uid);
    #endif

    //init neighbors
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        mydata->neighbors[i] = (Neighbor_t) {
            .uid = NO_ID,
            .symbolic_id = NO_ID,
            .distance = 0,
            .distance_prev = 0,
            .timexp_forget = 0};

    //initialize message receive buffer
    RB_init();
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
char *cb_botinfo(void)
{
    char *p = botinfo_buffer;
    p += sprintf (p, "ID: %d, MOTION: %s, COLOR: %s \n", kilo_uid, motionNames[mydata->current_motion_state], colorNames[mydata->current_led_color]);

    p += sprintf (p, "\n nr_neighbors = %d ", mydata->nr_neighbors);
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        if (mydata->neighbors[i].uid != NO_ID)
            p += sprintf (p, "n[%d]={%d, %d}, ", i, mydata->neighbors[i].uid, mydata->neighbors[i].distance);
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
