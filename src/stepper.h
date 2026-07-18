#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);

// Phase-stepping hooks (src/phase_exec.c)
struct stepper;
struct stepper *stepper_oid_lookup(uint8_t oid);
uint32_t stepper_get_subphase(struct stepper *s, uint32_t now);

#endif // stepper.h
