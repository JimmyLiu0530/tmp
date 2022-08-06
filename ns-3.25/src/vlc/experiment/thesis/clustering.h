/*
 * clustering.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_FUNCTION_CLUSTERING_H_
#define SCRATCH_FUNCTION_CLUSTERING_H_

#include "../thesis/parameter-config.h"

using namespace std;

namespace ns3 {

	//map<uint16_t, map<uint16_t, double>> CreateChannelMatrixM(Cluster &cluster);
	vector<vector<double>> CreateChannelMatrix(Cluster &cluster);

	double EstimatedChannelGain(Ptr<VlcTxNetDevice> Ap, Ptr<VlcRxNetDevice> Ue);

	vector<Cluster> Clustering(NodeContainer ap, NodeContainer ue, double da, double du, string ue_formation_method, string ue_init_method,
			string ap_formation_method, string on_off_ap_method, double index, vector<Cluster>(*ClusteringAlgo) (vector<Ptr<Node>>&, vector<Ptr<Node>>&, double, double,
					string, string, string, string, double));

	vector<Cluster> ClusteringAlgo(vector<Ptr<Node>> &candidate_ap, vector<Ptr<Node>> &candidate_ue,
			double da, double du, string ue_formation_method, string ue_init_method,
			string ap_formation_method, string on_off_ap_method, double index);

	void UeFormation(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ue,
			string method, string init_method, uint16_t du);
	void UeFormationKmeans(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ue,
			string method, uint16_t k);

	void ApAnchoring(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ap);
	void ApFormation(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ap,
			string method, uint16_t da);
	void OnOffAp(vector<Cluster> &clusters, string method, double index = 1);

	void InitUeSelection(Cluster &cluster, vector<Ptr<Node>> &candidate_ue);

	Ptr<Node> FarthestUeSelection(vector<Cluster> &clusters, Cluster &cluster,
			vector<Ptr<Node>> &candidate_ue, string method);

	Vector UpdateCentroid(vector<Ptr<Node>> &devices);

	//return whether the seed is within du
	bool NearestSeed(Cluster &cluster, vector<Ptr<Node>> &devices,
			string method, uint16_t du);

	//edge-distance
	//node-distance

	//UE Formation
	//AP Anchoring
	//AP Formation
	//On-Off AP
}



#endif /* SCRATCH_FUNCTION_CLUSTERING_H_ */
