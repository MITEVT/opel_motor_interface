#include "dmoc645.h"


int8_t DMOC_Decode_HV_Status(CCAN_MSG_OBJ_T *msg_obj, DMOC_HV_STAT_T *hv_stat) {
	if (msg_obj->mode_id != DMOC_HV_STAT_ID) return -1;

	uint16_t voltage = (msg_obj->data[0] << 8) | msg_obj->data[1];
	int16_t current = ((msg_obj->data[2] << 8) | msg_obj->data[3]) - 5000;

	hv_stat->hv_voltage = voltage;
	hv_stat->hv_current = current;

	return 0;
}

int32_t DMOC_Decode_Torque_Status(CCAN_MSG_OBJ_T *msg_obj) {
	if (msg_obj->mode_id != DMOC_TORQUE_STAT_ID) return -1;

	return ((msg_obj->data[0] << 8) | msg_obj->data[1]) - 30000;
}

int8_t DMOC_Decode_State(CCAN_MSG_OBJ_T *msg_obj, DMOC_OP_STATE_T *state) {
	if (msg_obj->mode_id != DMOC_STATE_ID) return -1;

	state->speed = ((msg_obj->data[0] << 8) | msg_obj->data[1]) - 20000;
	switch (msg_obj->data[6] >> 4) {
		case 0:
			state->op_stat = POWERUP;
			break;
		case 1:
			state->op_stat = DISABLED;
			break;
		case 2:
			state->op_stat = STANDBY;
			break;
		case 3:
			state->op_stat = ENABLED;
			break;
		case 4:
			state->op_stat = POWERDOWN;
			break;
		case 5:
			state->op_stat = FAULT;
			break;
		case 6:
			state->op_stat = CRITICAL_FAULT;
			break;
	}
	return 0;
}

uint8_t DMOC_Checksum(CCAN_MSG_OBJ_T msg) {
    uint8_t cs;
    uint8_t i;
    cs = msg.mode_id;
    for (i = 0; i < 7; i++)
        cs += msg.data[i];
    i = cs + 3;
    cs = ((int) 256 - i);
    return cs;
}
