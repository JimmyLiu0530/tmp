/*
 * resource-alloc.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_RESOURCE_ALLOC_H_
#define SCRATCH_RESOURCE_ALLOC_H_

#include "../thesis/parameter-config.h"

using namespace std;

namespace ns3 {
	struct Knapsack {
		double EE;
		map<uint16_t, VlcErrorModel::ModScheme> mod_order; 		//ue to order
		map<uint16_t, double> power; // ue to power
	};

	double ResourceAllocation(vector<Cluster> &clusters, uint16_t interf_iteration_num,
			string vt_ap_iteration_method, string interf_calc);
	double OnOffAP(vector<Cluster> &clusters, double ee, uint16_t interf_iteration_num,
			string vt_ap_iteration_method, string interf_calc, string method);
	vector< map<uint16_t, map<uint16_t, double> > > Precoding(vector<Cluster> &clusters);

	Knapsack VT_RA_Algorithm(vector<Cluster> clusters,
			Cluster cluster, double capacity, string vt_ap_iteration_method, string interf_calc);
	Knapsack CT_RA_Algorithm(vector<Cluster> clusters,
			Cluster cluster, double capacity, string interf_calc);

	double EstimatedInterference(vector<Cluster> &clusters, Cluster cluster,
			Ptr<Node> ue, string method = "approximate");
	double EstimatedThroughput(double order);

	bool BackhaulConstraintSatisfied(Knapsack knapsack, double Rmax);
	bool PowerConstraintSatisfied(Knapsack knapsack, Cluster cluster);

	vector<Cluster> CellSizeSorting(vector<Cluster> &clusters);

	double MinPowerRequired(double order, double interference);
	double MaxTransmitPower(map<uint16_t, map<uint16_t, double> > precoding_matrix,
			string method = "approximate");

	void ModOrderSelectionAndPowerAlloc(Cluster cluster,
			map<uint16_t, VlcErrorModel::ModScheme> m_order,
			map<uint16_t, double> power);

	//double ErfcInv(double x);

	bool CompareKnapsack(const Knapsack k1, const Knapsack k2);
	bool CompareAP(const pair<uint16_t, double> &a1, const pair<uint16_t, double> &a2);
	bool CompareClusterSize(const Cluster c1, const Cluster c2);
}



#endif /* SCRATCH_RESOURCE_ALLOC_H_ */
