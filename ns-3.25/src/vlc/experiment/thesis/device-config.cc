#include "../thesis/device-config.h"

#include "time.h"
#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "../thesis/parameter-config.h"
#include "ns3/core-module.h"
#include "ns3/vlc-device-helper.h"

using namespace std;

namespace ns3 {
	void ApConfig(NodeContainer &ap, double Rmax) {
		MobilityHelper mobility;
		Ptr < ListPositionAllocator > m_listPosition_ap = CreateObject<
				ListPositionAllocator>();
		//room size: 15m x 15m x 3m
		//APs are located uniformly distributed
		for(double x = 1.0; x <= sqrt(ap_num); x++) {
			for(double y = 1.0; y <= sqrt(ap_num); y++) {
				m_listPosition_ap->Add(Vector(length/(sqrt(ap_num)+1)*x, width/(sqrt(ap_num)+1)*y, height));
			}
		}

		mobility.SetPositionAllocator(m_listPosition_ap);
		mobility.SetMobilityModel("ns3::VlcMobilityModel");
		mobility.Install(ap);

		/*Set parameters for each AP and UE*/

		VlcDeviceHelper devHelper;
		for(uint16_t ap_i = 0; ap_i < ap_num; ap_i++) {
			std::string devName = std::to_string(ap_i); //should enable C++11
			devHelper.CreateTransmitter(devName);
			devHelper.SetTxMobility(devName, ap.Get(ap_i)->GetObject<VlcMobilityModel>());
			devHelper.SetTrasmitterParameter(devName, "pMax",Pmax);
			devHelper.SetTrasmitterParameter(devName, "SemiAngle", semi_angle);
			devHelper.SetTrasmitterParameter(devName, "Azimuth", azimuth);
			devHelper.SetTrasmitterParameter(devName, "Elevation",elevation);
			devHelper.SetTrasmitterParameter(devName, "Gain", gain); //TX Gain
			Vector ap_position = m_listPosition_ap->GetPosition(ap_i);
			Vector device_position = Vector(ap_position.x, ap_position.y, ap_height);
			devHelper.SetTrasmitterPosition(devName, device_position);
			devHelper.SetTrasmitterParameter(devName, "RMax", Rmax);

			ap.Get(ap_i)->AddDevice(devHelper.GetTransmitter(devName));
		}
	}
	void UeConfig(NodeContainer &ue, uint16_t ue_num, double fov) {
		MobilityHelper mobility;
		Ptr < ListPositionAllocator > m_listPosition_ue = CreateObject<
				ListPositionAllocator>();

		//UEs are located randomly
		double min_bound = 0.0, max_bound = 15.0;
		for(uint16_t i = 0; i < ue_num; i++) {
			float x, y;
			x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/length));
			y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/width));

			m_listPosition_ue->Add(Vector(x, y, 0.0));
		}
		mobility.SetPositionAllocator(m_listPosition_ue);
		mobility.SetMobilityModel("ns3::VlcMobilityModel");
		mobility.Install(ue);

		VlcDeviceHelper devHelper;
		for(uint16_t ue_i = 0; ue_i < ue_num; ue_i ++) {
			std::string devName = std::to_string(ue_i); //should enable C++11
			devHelper.CreateReceiver(devName);
			devHelper.SetReceiverParameter(devName, "FilterGain", filter_gain);
			devHelper.SetReceiverParameter(devName, "RefractiveIndex", refractive_index);
			devHelper.SetReceiverParameter(devName, "FOVAngle", fov);
			devHelper.SetReceiverParameter(devName, "ConcentrationGain", concentration_gain);
			devHelper.SetReceiverParameter(devName, "PhotoDetectorArea", A);
			devHelper.SetReceiverParameter(devName, "RXGain", rxgain); //cosine of incidence
			devHelper.SetReceiverParameter(devName, "Beta", beta);
			devHelper.SetReceiverParameter(devName, "Responsivity", res);
			/****************************************MODULATION SCHEME SETTINGS******************
			*AVILABLE MODULATION SCHEMES ARE: [1]. VlcErrorModel::PAM4, [2]. VlcErrorModel::OOK [3]. VlcErrorModel::VPPM
			*AVILABLE MODULATION SCHEMES [4]. VlcErrorModel::PSK4 [5]. VlcErrorModel::PSK16. [6]. VlcErrorModel::QAM4 [7]. VlcErrorModel::QAM16.
			************************************************************************************/
			devHelper.SetReceiverParameter(devName, "SetModulationScheme",VlcErrorModel::PSK4);
			//devHelper.SetReceiverParameter(devName, "DutyCycle", 0.85);
			devHelper.SetRxMobility(devName, ue.Get(ue_i)->GetObject<VlcMobilityModel>());
			Vector ue_position = m_listPosition_ue->GetPosition(ue_i);
			Vector device_position = Vector(ue_position.x, ue_position.y, ue_height);
			devHelper.SetReceiverPosition(devName, device_position);

			ue.Get(ue_i)->AddDevice(devHelper.GetReceiver(devName));

			//install mobility model
			/*ue.Get(ue_i)->GetObject<VlcMobilityModel>()->SetVelocityAndAcceleration(
					Vector(1.0, 1.0, 0.0), Vector(0.0, 0.0, 0.0));*/
		}
	}
}
