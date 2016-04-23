#include "dmoc645.h"


int8_t DMOC_Decode_HV_Status(CCAN_MSG_OBJ_T *msg_obj, DMOC_HV_STAT_T *hv_stat) {
	if (msg_obj->mode_id != DMOC_HV_STAT_ID) return -1;

	uint16_t voltage = (msg_obj->data[0] << 8) | msg_obj->data[1];
	int16_t current = ((msg_obj->data[2] << 8) | msg_obj->data[3]) - 5000;

	hv_stat->hv_voltage = voltage;
	hv_stat->hv_current = current;

	return 0;
}

int32_t DMOC_Decode_Torque_Status(CCAN_MSG_OBJ, *msg_obj) {
	if (msg_obj->mode_id != DMOC_TORQUE_STAT_ID) return -1;

	return ((msg_obj->data[0] << 8) | msg_obj->data[1]) - 30000;
}