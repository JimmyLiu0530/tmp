#include "../thesis/transmission.h"

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

map<string, Ptr<VlcChannel>> ChannelEstablishment(vector<Cluster> &clusters) {

		VlcChannelHelper chHelper;
		map<string, Ptr<VlcChannel>> channels;
		int name = 0;

		for(vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();
				++it) {
			for(vector<Ptr<Node>>::iterator it_ap = it->AP.begin(); it_ap != it->AP.end();
					++it_ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*it_ap)->GetDevice(0));
				for(vector<Ptr<Node>>::iterator it_ue = it->UE.begin(); it_ue != it->UE.end();
						++it_ue) {
					Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*it_ue)->GetDevice(0));
					string chName = to_string(name);

					if(Ue->GetDeviceStatus()) {
					//	cout << (*it_ue)->GetId() << endl;
						chHelper.CreateChannel(chName);
						chHelper.SetPropagationLoss(chName, "VlcPropagationLoss");
						chHelper.SetPropagationDelay(chName, delay);
						chHelper.AttachTransmitter(chName, Ap);
						chHelper.AttachReceiver(chName, Ue);
						chHelper.SetChannelParameter(chName, "TEMP", temp);
						chHelper.SetChannelParameter(chName, "BAND_FACTOR_NOISE_SIGNAL",Band_factor_Noise_Signal );
						chHelper.SetChannelWavelength(chName, wl_low, wl_high);
						chHelper.SetChannelParameter(chName, "ElectricNoiseBandWidth",bw);

						// And then install devices and channels connecting our topology.
						channels[chName] = chHelper.InstallwithInterference(*it_ap, *it_ue, chName);
						//channels[chName] = chHelper.Install(chName, *it_ap, *it_ue);

						name++;

						//cout << (*it_ap)->GetId() << " " << (*it_ue)->GetId() << endl;
					}
				}
			}
		}

		return channels;
	}

	vector<Ipv4InterfaceContainer> InternetStackInstall(map<string, Ptr<VlcChannel>> &channels) {

		vector<Ipv4InterfaceContainer> ipInterfs;

		InternetStackHelper internet;
		internet.InstallAll();

		int count = 1;
		int first, second;

		// Later, we add IP addresses.
		Ipv4AddressHelper ipv4;
		for(map<string, Ptr<VlcChannel>> ::iterator channel = channels.begin();
				channel != channels.end(); ++channel) {
			NetDeviceContainer devices;
			devices.Add(channel->second->GetDevice(0));
			devices.Add(channel->second->GetDevice(1));

			std::ostringstream subnet;
			first = count / 256;
			second = count % 256;
			subnet << "10." << first << "." << second << ".0";
			ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
			ipInterfs.push_back(ipv4.Assign(devices));

			count++;
		}
		// and setup ip routing tables to get total ip-level connectivity.
		//Ipv4GlobalRoutingHelper::PopulateRoutingTables();

			///////////////////////////////////////////////////////////////////////////
			// Simulation 1
			//
			// Send 2000000 bytes over a connection to server port 50000 at time 0
			// Should observe SYN exchange, a lot of data segments and ACKS, and FIN
			// exchange.  FIN exchange isn't quite compliant with TCP spec (see release
			// notes for more info)
			//
			///////////////////////////////////////////////////////////////////////////

		return ipInterfs;
	}

	vector<Ptr<Socket>> ApplicationInstall(map<string, Ptr<VlcChannel>> &channels) {
		uint16_t servPort = 4000;

		// Create a packet sink to receive these packets on n2...
		PacketSinkHelper sink("ns3::TcpSocketFactory",
				InetSocketAddress(Ipv4Address::GetAny(), servPort));

		ApplicationContainer apps;
		for(map<string, Ptr<VlcChannel>>::iterator it = channels.begin();
							it!=channels.end(); it++) {
			apps.Add(sink.Install(it->second->GetDevice(1)->GetNode()));

		}
		apps.Start(Seconds(0.0));
		apps.Stop(Seconds(1000.0));
				// Create a source to send packets from n0.  Instead of a full Application
				// and the helper APIs you might see in other example files, this example
				// will use sockets directly and register some socket callbacks as a sending
				// "Application".
				// Create and bind the socket...

		vector<Ptr<Socket>> localsockets;

		//for each channel
		for(map<string, Ptr<VlcChannel>>::iterator it = channels.begin();
							it!=channels.end(); it++) {
			Ptr<Node> tx = it->second->GetDevice(0)->GetNode();
			Ptr < Socket > localSocket = Socket::CreateSocket(tx,
										TcpSocketFactory::GetTypeId());
			localSocket->Bind();
			localsockets.push_back(localSocket);
		}

		return localsockets;
	}

	void ResultOutput() {
/*
			DataTransmission(clusters);

			Simulator::Stop(Seconds(1000.0));
			Simulator::Run();

			for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
					++cluster) {
				for(vector<Ptr<Node>>::iterator Ue = cluster->UE.begin(); Ue != cluster->UE.end();
						++Ue) {
					ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> ((*Ue)->GetDevice(0));

					if(rx->GetTime() > 0){
						rate += rx->ComputeGoodPut() * 8;
						time += rx->GetTime();
						if(rx->GetTime() > max_time)
							max_time = rx->GetTime();
					}

				}
				for(vector<Ptr<Node>>::iterator Ap = cluster->AP.begin(); Ap != cluster->AP.end();
						++Ap) {
					ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> ((*Ap)->GetDevice(0));

					power += tx->GetTotalPowerConsumption();

				}
			}

			cout << i << ": " << endl;
			if(rate != 0) {
				total_ee += rate / max_time / power;
				cout << "EE per active UE = " << rate / time / power << endl;
				cout << "total EE = " << rate / max_time / power << endl;
				cout << "rate = " << rate / time << endl;
				cout << "power = " << power << endl;
			}

			Simulator::Destroy();
		}
		cout << "average total EE = " << total_ee / iteration << endl;
		fout << "average total EE = " << total_ee / iteration << endl;*/
	}


}
