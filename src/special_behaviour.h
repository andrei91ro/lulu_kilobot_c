// vim:filetype=c
/**
 * @file special_behaviour.h
 * @brief Lulu_kilobot extension that allows certain robots (specified using the special_kilo_uid[] array) to call a specified setup() and loop() function, different from that used by normal P colony controlled Kilobots
 * @author Andrei G. Florea
 * @author Catalin Buiu
 * @date 2017-03-29
 */
#ifndef SPECIAL_BEHAVIOUR_H
#define SPECIAL_BEHAVIOUR_H
#include "lulu_kilobot.h" //for access to sensors, motors (includes kilolib.h)

extern uint16_t size_special_kilo_uid;
#ifdef SIMULATOR
    extern uint16_t special_kilo_uid[];
#else
    extern uint16_t special_kilo_uid_physical[];
#endif

/**
 * @brief Setup special message contents so that messages can be sent
 */
void setup_special_message(USERDATA *mydata);
void setup_special(USERDATA *mydata);
void loop_special(USERDATA *mydata);

#endif
