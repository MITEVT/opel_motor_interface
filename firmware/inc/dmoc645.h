#include "chip.h"

#ifndef _DMOC645_H_
#define _DMOC645_H_

typedef enum {POWERUP, OFF, ON, FAULT} DMOC_FSM_STATE_T;

#define DMOC_HV_STAT_ID 0x650

typedef struct {
	uint16_t hv_voltage;
	int16_t hv_current;
} DMOC_HV_STAT_T;

int8_t DMOC_Decode_HV_Status(CCAN_MSG_OBJ_T *msg_obj, DMOC_HV_STAT_T *hv_stat);

#define DMOC_TEMP_STAT_ID 0x651
#define DMOC_TORQUE_STAT_ID 0x23A

int32_t DMOC_Decode_Torque_Status(CCAN_MSG_OBJ_T *msg_obj);

#define DMOC_STATE_ID 0x23B

typedef struct {
	DMOC_FSM_STATE_T op_stat;
	int16_t speed;
} DMOC_OP_STATE_T;

int8_t DMOC_Decode_State(CCAN_MSG_OBJ_T *msg_obj, DMOC_OP_STATE_T *state);

#define DMOC_ELEC_STATE_ID 0x23E

uint8_t DMOC_Checksum(CCAN_MSG_OBJ_T msg);

#endif
