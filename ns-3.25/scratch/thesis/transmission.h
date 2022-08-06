/*
 * transmission.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_FUNCTION_TRANSMISSION_H_
#define SCRATCH_FUNCTION_TRANSMISSION_H_

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "../thesis/parameter-config.h"

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
//#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"
#include "ns3/netanim-module.h"

using namespace std;

namespace ns3 {

	map<string, Ptr<VlcChannel>> ChannelEstablishment(vector<Cluster> &clusters);
	vector<Ipv4InterfaceContainer> InternetStackInstall(map<string, Ptr<VlcChannel>> &channels);
	vector<Ptr<Socket>> ApplicationInstall(map<string, Ptr<VlcChannel>> &channels);

	void ResultOutput();

}




#endif /* SCRATCH_FUNCTION_TRANSMISSION_H_ */
