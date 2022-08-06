/*
 * device-config.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_FUNCTION_DEVICE_CONFIG_H_
#define SCRATCH_FUNCTION_DEVICE_CONFIG_H_

#include "ns3/core-module.h"
#include "ns3/vlc-device-helper.h"

namespace ns3 {
	void ApConfig(NodeContainer &ap, double Rmax);
	void UeConfig(NodeContainer &ue, uint16_t ue_num, double fov);

	void SetModulationOrder();
}

#endif /* SCRATCH_FUNCTION_DEVICE_CONFIG_H_ */
