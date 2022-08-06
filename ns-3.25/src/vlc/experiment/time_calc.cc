#include <iostream>
#include <time.h>

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "../thesis/device-config.h"
#include "../thesis/parameter-config.h"
#include "../thesis/clustering.h"
#include "../thesis/resource-alloc.h"
#include "../thesis/transmission.h"
#include "../thesis/matrix.h"

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
//#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"
#include "ns3/netanim-module.h"

#include "time.h"
#include <sys/stat.h>

using namespace ns3;

int main() {
	double t = 0;

	for(int i = 0; i < 20; ++i) {
		//  LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
		srand(time(0));

		//***************INPUTS**************//
		double du = 5, da = 200;
		string ue_formation_method = "dynamic", ue_init_method = "random", ap_formation_method = "average-channel-gain";
		string on_off_ap_method = "OffSome";
		double index = 2;
		string vt_ap_iteration_method = "average";
		string interf_calc = "approximate";
		uint16_t interf_iteration_num = 1;
		uint16_t ue_num = 10;
		double Rmax = 1e100, fov = 65;

		//***************INPUTS**************//

		NodeContainer ap, ue;
		ap.Create(ap_num);
		ue.Create(ue_num);

		//Config APs and UEs
		ApConfig(ap, Rmax);
		UeConfig(ue, ue_num, fov);

		double start, end;

		//Run clustering algorithm
		vector<Cluster> clusters = Clustering(ap, ue, da, du, ue_formation_method,
				ue_init_method, ap_formation_method, on_off_ap_method, index, ClusteringAlgo);

		start = clock();

		//Run RA algorithm
		double ee = ResourceAllocation(clusters, interf_iteration_num, vt_ap_iteration_method,
				interf_calc);
		if(on_off_ap_method == "exhaustive")
			OnOffAP(clusters, ee, interf_iteration_num, vt_ap_iteration_method,
					interf_calc);

		end = clock();

		t += (end - start) / CLOCKS_PER_SEC * 1000;

		std::cout << t << "ms" << std::endl;

		Simulator::Stop(Seconds(1000.0));
		Simulator::Run();

		Simulator::Destroy();
	}

	cout << t/20 << endl;

	return 0;
}
