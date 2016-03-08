// vim:filetype=c
/**
 * @file lulu_kilobot.h
 * @brief Lulu P colony simulator extension that allows controlling a Kilobot robot by using a P colony based controller
 * In this header we declare the structure and functions of a basic Kilobot controller that is based on P colonies and implements a bidirectional symbolic object - numeric value conversion.
 * This application uses the Kilombo simulator and also the kilolib library for interfacing with both a simulated and real kilobot.
 * @author Andrei G. Florea
 * @author Catalin Buiu
 * @date 2016-03-06
 */

#ifndef LULU_KILOBOT_H
#define LULU_KILOBOT_H

#include <kilombo.h> //for access to sensors, motors (includes kilolib.h)
#include "lulu.h"

/**
 * @brief Define motion types
 */
typedef enum {
    MOTION_STOP,
    MOTION_STRAIGHT,
    MOTION_LEFT,
    MOTION_RIGHT
} motion_t;

typedef enum {
    COLOR_OFF,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    //TODO add all basic colors used in lulu_kilobot.py
} led_color_t;

uint8_t colorValues[] = {RGB(0, 0, 0), RGB(3, 0, 0), RGB(0, 3, 0), RGB(0, 0, 3)};

typedef struct {
    Pcolony_t pcol;
    uint8_t light,
            light_prev;
    motion_t current_motion_state;
    led_color_t current_led_color;
    sim_step_result_t sim_result;

} USERDATA;

#ifdef SIMULATOR
    /* provide a text string for the simulator status bar about this bot */
    static char botinfo_buffer[10000];
    extern char* motionNames[];
    extern char* colorNames[];
#endif

//TODO params need tunning
#define PARAM_LIGHT_THRESHOLD 20
#define PARAM_DISTANCE_THRESHOLD 55

/**
 * @brief Process raw_state info received from sensors and populate the input module agents with significant objects
 *
 * @param data USERDATA structure that contains all relevant information about this Kilobot
 */
void procInputModule(USERDATA *data);

/**
 * @brief Process the objects present in the output module agents and transform them into commands that are executed by the Kilobot
 *
 * @param data USERDATA structure that contains all relevant information about this Kilobot
 */
void procOutputModule(USERDATA *data);

/**
 * @brief Process sensor data and transfer it to the input data structures for further processing and symbolic conversion
 *
 * @param data The USERDATA structure where the processed data will be stored
 * @see procInputModule
 */
void getState(USERDATA *data);

/**
 * @brief Set the motion direction
 *
 * @param dir_new The new motion direction (DIR_{STOP, FORWARD, LEFT, RIGHT})
 */
void set_motion(motion_t dir_new);

/**
 * @brief Function called after receiving a new message
 *
 * @param _msg Pointer to the new message
 * @param d Structure that allows to determine the distance from the sender
 */
void message_rx(message_t* _msg, distance_measurement_t* d);

/**
 * @brief Function called when preparing a new message
 *
 * @return Pointer to the new message
 */
message_t* message_tx();

/**
 * @brief Function called after the message has been succesfully sent
 */
void message_tx_success();

/**
 * @brief Function called in a continuos loop during the runtine of the program
 */
void loop();

/**
 * @brief Function called only once at startup, before entering the continuos loop
 */
void setup();

#ifdef SIMULATOR
/**
 * @brief Prepares a status bar message for the simulator
 *
 * @return The status bar message
 */
char *cb_botinfo(void);
#endif

#endif
