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
#include "ns3/netanim-module.h"

#include "../thesis/parameter-config.h"
#include "../thesis/clustering.h"
#include "../thesis/matrix.h"

using namespace std;

namespace ns3 {
	/*map<uint16_t, map<uint16_t, double>> CreateChannelMatrixM(Cluster &cluster) {
		//calculated at central unit, all the data are "estimated", not real
		//Initialize the channel matrix

		//UE to AP
		map<uint16_t, map<uint16_t, double>> channel_matrix;

		for(vector<Ptr<Node>>::iterator it_ue = cluster.UE.begin();
				it_ue != cluster.UE.end(); ++it_ue) {

			Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
									(*it_ue)->GetDevice(0));
			uint16_t ue_id = (*it_ue)->GetId();

			for(vector<Ptr<Node>>::iterator it_ap = cluster.AP.begin();
					it_ap != cluster.AP.end(); ++it_ap) {

				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
						(*it_ap)->GetDevice(0));

				double dist = CalculateDistance(Ap->GetPosition(),
						Ue->GetPosition());
				double TxGain = Ap->GetTXGain();
				double RxGain = Ue->GetRXGain();
				//concentration gain
				double cGain;
				double incidence_sin = height / dist;
				double fov_sin = sin(Ue->GetFOVAngle());

				if(incidence_sin <= fov_sin)
					cGain = Ue->GetConcentrationGain();
				else
					cGain = 0;

				double ch_gain = TxGain * RxGain * filter_gain *
								cGain * A / pow(dist, 2);

				uint16_t ap_id = (*it_ap)->GetId();

				channel_matrix[ue_id].insert(pair<uint16_t, double>(ap_id, ch_gain));
			}

		}

		return channel_matrix;
	}*/

	vector<vector<double>> CreateChannelMatrix(Cluster &cluster) {
		//calculated at central unit, all the data are "estimated", not real
		//Initialize the channel matrix

		//UE to AP
		vector<vector<double>> channel_matrix;

		for(vector<Ptr<Node>>::iterator it_ue = cluster.UE.begin();
				it_ue != cluster.UE.end(); ++it_ue) {
			Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
									(*it_ue)->GetDevice(0));

			vector<double> tmp;

			for(vector<Ptr<Node>>::iterator it_ap = cluster.AP.begin();
					it_ap != cluster.AP.end(); ++it_ap) {

				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
						(*it_ap)->GetDevice(0));

				double ch_gain = EstimatedChannelGain(Ap, Ue);
				//cout << ch_gain << " ";

				tmp.push_back(ch_gain);
			}
			//cout << endl;
			channel_matrix.push_back(tmp);
			tmp.clear();
		}

		//PrintOutMatrix(channel_matrix);

		return channel_matrix;
	}

	double EstimatedChannelGain(Ptr<VlcTxNetDevice> Ap, Ptr<VlcRxNetDevice> Ue) {
		Vector AP_pos = Ap->GetPosition();
		Vector UE_pos = Ue->GetPosition();

		double dist = CalculateDistance(AP_pos,
				UE_pos);

		double angle;

		double height_diff = AP_pos.z - UE_pos.z;
		double plane_dist = sqrt(pow(AP_pos.x - UE_pos.x, 2) + pow(AP_pos.y - UE_pos.y, 2));
		double hypotenuse = dist;

		 // angle of incidence = angle between <height diff> and <hypotenuse>
		angle = acos((pow(height_diff,2) + pow(hypotenuse,2) - pow(plane_dist,2)) / (2*height_diff*hypotenuse))
						* 180 / M_PI;
		Ap->SetAngleOfRadiance(angle);
		Ue->SetIncidenceAngle(angle);
		double TxGain = Ap->GetTXGain();
		double RxGain = Ue->GetRXGain();
		//concentration gain
		double cGain = Ue->GetConcentrationGain();

		double ch_gain = TxGain * RxGain * filter_gain *
						cGain * A / pow(dist, 2);

		return ch_gain;
	}

	vector<Cluster> Clustering(NodeContainer ap, NodeContainer ue, double da, double du,
			string ue_formation_method, string ue_init_method, string ap_formation_method,
			string on_off_ap_method, double index, vector<Cluster>(*ClusteringAlgo) (vector<Ptr<Node>>&, vector<Ptr<Node>>&, double, double,
					string, string, string, string, double)) {

		vector<Ptr<Node>> AP, UE;
		for(NodeContainer::Iterator it = ap.Begin(); it != ap.End(); ++it) {
			AP.push_back(*it);
		}
		for(NodeContainer::Iterator it = ue.Begin(); it != ue.End(); ++it) {
			UE.push_back(*it);
		}

		return ClusteringAlgo(AP, UE, da, du, ue_formation_method, ue_init_method,
				ap_formation_method, on_off_ap_method, index);
	}

	vector<Cluster> ClusteringAlgo(vector<Ptr<Node>> &candidate_ap, vector<Ptr<Node>> &candidate_ue,
			double da, double du, string ue_formation_method, string ue_init_method,
			string ap_formation_method, string on_off_ap_method, double index) {

		vector<Cluster> clusters;

		UeFormation(clusters, candidate_ue, ue_formation_method, ue_init_method, du);
		//UeFormationKmeans(clusters, candidate_ue, "centroid", 5);
		ApAnchoring(clusters, candidate_ap);
		ApFormation(clusters, candidate_ap, ap_formation_method, da);
		//wrong clustering exception
		map<uint16_t, uint16_t> duplicates; //check whether there are duplicate APs/UEs
		OnOffAp(clusters, on_off_ap_method, index);

		//////////////////////////////////

		/*cout << "Anchoring AP set" << endl;
		for(std::vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			for(std::vector<Ptr<Node>>::iterator it_ap = cluster->anchoring_ap.begin();
					it_ap != cluster->anchoring_ap.end(); ++it_ap) {
				cout << (*it_ap)->GetId() << " ";
			}
			cout << endl;
		}*/

		cout << "AP set" << endl;
		for(std::vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			for(std::vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin();
					it_ap != cluster->AP.end(); ++it_ap) {
				cout << (*it_ap)->GetId() << " ";
			}
			cout << endl;
		}

		cout << "UE set" << endl;
		for(std::vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			for(std::vector<Ptr<Node>>::iterator it_ue = cluster->UE.begin();
					it_ue != cluster->UE.end(); ++it_ue) {
				cout << (*it_ue)->GetId() << " ";
			}
			cout << endl;
		}

		for(std::vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			for(std::vector<Ptr<Node>>::iterator it_ue = cluster->UE.begin();
					it_ue != cluster->UE.end(); ++it_ue) {
				uint16_t id = (*it_ue)->GetId();
				map<uint16_t, uint16_t>::iterator check = duplicates.find(id);
				if(check != duplicates.end())
					throw std::logic_error("Error in clustering algorithm!");
				duplicates[id] = id;
			}
			for(std::vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin();
					it_ap != cluster->AP.end(); ++it_ap) {
				uint16_t id = (*it_ap)->GetId();
				map<uint16_t, uint16_t>::iterator check = duplicates.find(id);
				if(check != duplicates.end())
					throw std::logic_error("Error in clustering algorithm!");
				duplicates[id] = id;
			}
		}

		for(std::vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			if(cluster->AP.size() * cluster->UE.size() > 200) {
				throw std::logic_error("timeout");
			}
		}

		return clusters;

	}

	void UeFormation(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ue,
			string method, string init_method, uint16_t du) {
		uint16_t id = 0;
		//UE Formation
		if(method == "centroid") {
			while(!candidate_ue.empty()) {
				Cluster cluster;

				//init: select a random UE as centroid
				if(init_method == "random")
					InitUeSelection(cluster, candidate_ue);
				else if(init_method == "farthest") {
					if(id == 0)
						InitUeSelection(cluster, candidate_ue);
					else
						FarthestUeSelection(clusters, cluster, candidate_ue, method);
				}
				else {
					cout << init_method << endl;
					throw std::logic_error("Method not exists");
				}


				while(1) {
					//pick a nearest UE and recruit it to the cluster
					//according to centroid-distance, or edge-distance, or node-distance
					if(!NearestSeed(cluster, candidate_ue, method, du))
						break;

						//update centroid
						cluster.centroid = UpdateCentroid(cluster.UE);

				}
				cluster.ID = id;
				id++;
				clusters.push_back(cluster);
			}
		}
		else if(method == "dynamic") {

			//Stage I: Determine staring seeds

			//Step 1: Determine the first seed
			Cluster cluster;
			cluster.ID = id;

			Vector centroid = UpdateCentroid(candidate_ue);

			double max_dist = 0;
			Ptr<Node> starting_seed;
			Vector vec;
			vector<Ptr<Node>>::iterator to_erase;

			for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin(); ue != candidate_ue.end();
					++ue) {

				Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));
				double dist = CalculateDistance(centroid, Ue->GetPosition());

				if(dist > max_dist) {
					max_dist = dist;
					starting_seed = *ue;
					vec = Ue->GetPosition();
					to_erase = ue;
				}

			}

			cluster.UE.push_back(starting_seed);
			cluster.centroid = vec;
			clusters.push_back(cluster);
			candidate_ue.erase(to_erase);

			//Step 2: Determine the rest of the starting seeds

			bool finish = false;

			while(!finish) {
				Cluster new_cluster;
				Ptr<Node> seed = FarthestUeSelection(clusters, new_cluster, candidate_ue, "centroid");
				id += 1;
				new_cluster.ID = id;
				clusters.push_back(new_cluster);

				//calculate distance between each cell

				vector<Cluster>::iterator this_cluster = clusters.begin() + id;

				for(vector<Cluster>::iterator c = clusters.begin(); c != clusters.end() - 1;
						++c) {

					if(CalculateDistance(c->centroid, this_cluster->centroid) < du) {
						clusters.erase(this_cluster);
						finish = true;
						candidate_ue.push_back(seed);
						break;
					}
				}

				if(candidate_ue.empty())
					break;
			}

			//Stage II: assign the rest of each UE to its nearest cluster
			for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin(); ue != candidate_ue.end();
					++ue) {
				uint16_t selected_cluster_id = 0;
				Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));
				double dist;
				double min_dist = CalculateDistance(clusters.at(0).centroid, Ue->GetPosition());
				for(vector<Cluster>::iterator c = clusters.begin() + 1; c != clusters.end();
						++c) {
					dist = CalculateDistance(c->centroid, Ue->GetPosition());
					if(dist < min_dist) {
						min_dist = dist;
						selected_cluster_id = c->ID;
					}
				}
				clusters.at(selected_cluster_id).UE.push_back(*ue);
				clusters.at(selected_cluster_id).centroid = UpdateCentroid(clusters.at(selected_cluster_id).UE);
			}

		}

		else if(method == "k-means") {
			UeFormationKmeans(clusters, candidate_ue, "centroid", du);
		}

		else {
			cout << method << endl;
			throw std::logic_error("Method not exists");
		}

		//cout << clusters.size() << endl;
	}

	void UeFormationKmeans(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ue,
			string method, uint16_t k) {
		double max_dist_among_centroids = 0;
		vector<Ptr<Node>> all_ues = candidate_ue;
		vector<Cluster> sol;
		vector<Cluster> tmp;

		for(uint16_t i = 0; i < M; ++i) {
			candidate_ue = all_ues;

			//select k seeds
			Cluster cluster;
			InitUeSelection(cluster, candidate_ue);
			cluster.ID = 0;
			tmp.push_back(cluster);

			Ptr<Node> first_seed = tmp.at(0).UE.at(0);
			Ptr<VlcNetDevice> first_ue = DynamicCast <VlcNetDevice> (first_seed->GetDevice(0));

			double total_dist = 0;
			for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin();
					ue != candidate_ue.end(); ++ue) {
				Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));
				total_dist += CalculateDistance(first_ue->GetPosition(), Ue->GetPosition());
			}

			uint16_t seed_num = 0;

			while(seed_num != k - 1) {

				for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin();
						ue != candidate_ue.end();) {

					Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));
					double dist = CalculateDistance(first_ue->GetPosition(), Ue->GetPosition());
					double prob = dist / total_dist;

					double r = (double) rand() / RAND_MAX;
					if(r <= prob) {
						Cluster cluster;
						cluster.UE.push_back(*ue);
						seed_num+=1;
						cluster.ID = seed_num;
						//cout << (*ue)->GetId() << endl;

						if(method == "centroid")
							cluster.centroid = Ue->GetPosition();
						tmp.push_back(cluster);

						candidate_ue.erase(ue);

					}
					else
						ue++;

					if(seed_num == k - 1)
						break;
				}

			}

			for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin();
						ue != candidate_ue.end(); ++ue) {

				Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));

				double min_dist = CalculateDistance(Ue->GetPosition(),
						tmp.at(0).centroid);
				uint16_t cluster_id = 0;
				for(vector<Cluster>::iterator cluster = tmp.begin() + 1;
						cluster != tmp.end(); ++cluster) {
					double dist = CalculateDistance(Ue->GetPosition(),
							cluster->centroid);
					if(dist < min_dist) {
						min_dist = dist;
						cluster_id = cluster->ID;
					}
				}
				tmp.at(cluster_id).UE.push_back(*ue);
				tmp.at(cluster_id).centroid = UpdateCentroid(tmp.at(cluster_id).UE);
			}


			double dist = 1.0;
			for(vector<Cluster>::iterator cluster = tmp.begin();
						cluster != tmp.end() - 1; ++cluster) {
				for(vector<Cluster>::iterator cluster2 = cluster + 1;
										cluster2 != tmp.end(); ++cluster2) {
					dist *= CalculateDistance(cluster->centroid, cluster2->centroid);
				}
			}

			if(dist > max_dist_among_centroids) {
				max_dist_among_centroids = dist;
				sol = tmp;
			}

			tmp.clear();
		}

		clusters = sol;
	}

	void ApAnchoring(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ap) {
		//TODO: The code and algorithm are crap. Should be rewritten

		//AP anchoring
		vector<Ptr<Node>> all_ue;
		vector<Ptr<Node>> all_ap = candidate_ap;
		map<uint16_t, vector<Ptr<Node>>> anchoring;

		//init
		for(vector<Cluster>::iterator it_cluster = clusters.begin(); it_cluster != clusters.end();
				++it_cluster) {
			vector<bool> tmp;
			for(vector<Ptr<Node>>::iterator it_ue = it_cluster->UE.begin(); it_ue !=
					it_cluster->UE.end(); ++it_ue) {
				all_ue.push_back(*it_ue);
			}
		}

		map<uint16_t, Ptr<Node>> final_anchoring;

		while(!all_ue.empty()) {

			for(vector<Ptr<Node>>::iterator it_ue = all_ue.begin(); it_ue != all_ue.end();
					++it_ue) {
				Ptr<VlcNetDevice> ue = DynamicCast <VlcNetDevice> ((*it_ue)->GetDevice(0));
				Ptr<VlcNetDevice> ap0 = DynamicCast <VlcNetDevice> (candidate_ap[0]->GetDevice(0));
				double min_dist = CalculateDistance(ue->GetPosition(), ap0->GetPosition());
				uint32_t id = candidate_ap[0]->GetId();

				for(std::vector<Ptr<Node>>::iterator it_ap = candidate_ap.begin() + 1; it_ap != candidate_ap.end();
						++it_ap) {
					Ptr<VlcNetDevice> ap = DynamicCast <VlcNetDevice> ((*it_ap)->GetDevice(0));
					double dist = CalculateDistance(ue->GetPosition(), ap->GetPosition());
					if(dist < min_dist) {
						min_dist = dist;
						id = (*it_ap)->GetId();
					}
				}
				anchoring[id].push_back(*it_ue);
			}

			//check whether there are more than two UEs select the same AP
			for(map<uint16_t, std::vector<Ptr<Node>>>::iterator it = anchoring.begin();
					it != anchoring.end(); ++it) {

				if(it->second.size() > 1) {
					Ptr<VlcNetDevice> Ap = DynamicCast <VlcNetDevice>
						(all_ap[it->first]->GetDevice(0));
					Ptr<VlcNetDevice> ue0 = DynamicCast <VlcNetDevice>
						(it->second.at(0)->GetDevice(0));
					double min_dist = CalculateDistance(Ap->GetPosition(), ue0->GetPosition());
					Ptr<Node> selected = it->second.at(0);

					for(std::vector<Ptr<Node>>::iterator it_ue = it->second.begin() + 1;
							it_ue != it->second.end(); ++it_ue) {
						Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice>
							((*it_ue)->GetDevice(0));
						double dist = CalculateDistance(Ap->GetPosition(), Ue->GetPosition());
						if(dist < min_dist) {
							min_dist = dist;
							selected = *it_ue;
						}
					}
					it->second.clear();
					it->second.push_back(selected);
				}

				//remove UE
				vector<Ptr<Node>>::iterator to_remove_ue;
				for(vector<Ptr<Node>>::iterator find = all_ue.begin(); find != all_ue.end();
						++find) {
					if(*find == it->second.at(0)) {
						to_remove_ue = find;
						break;
					}
				}
				all_ue.erase(to_remove_ue);

				//remove AP
				vector<Ptr<Node>>::iterator to_remove_ap;
				for(vector<Ptr<Node>>::iterator it_ap = candidate_ap.begin(); it_ap != candidate_ap.end();
						++it_ap) {
					if(it->first == (*it_ap)->GetId()) {
						to_remove_ap = it_ap;
						break;
					}
				}
				candidate_ap.erase(to_remove_ap);
			}

			for(map<uint16_t, std::vector<Ptr<Node>>>::iterator it = anchoring.begin();
					it != anchoring.end(); ++it) {
				uint16_t id = it->second.at(0)->GetId();
				final_anchoring[id] = all_ap[it->first];
			}

			anchoring.clear();

		}

		//Add anchoring APs to corresponding cluster
		for(vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();
				++it) {

			for(vector<Ptr<Node>>::iterator it_ue = it->UE.begin(); it_ue != it->UE.end();
					++it_ue) {
				uint16_t id = (*it_ue)->GetId();
				it->AP.push_back(final_anchoring[id]);
				it->anchoring_ap.push_back(final_anchoring[id]);
			}
		}
	}

	void ApFormation(vector<Cluster> &clusters, vector<Ptr<Node>> &candidate_ap,
			string method, uint16_t da) {

		//AP Formation
		if(method == "centroid") {
			for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();
					++it) {
				Vector centroid_pos = it->centroid;

				//AP clustering

				for(vector<Ptr<Node>>::iterator it_ap = candidate_ap.begin(); it_ap != candidate_ap.end();
						++it_ap) {
					Ptr<VlcNetDevice> Ap = DynamicCast <VlcNetDevice> ((*it_ap)->GetDevice(0));
					double dist = CalculateDistance(centroid_pos, Ap->GetPosition());
					if(dist <= da) {
						it->AP.push_back(*it_ap);

					}
				}
			}
			OnOffAp(clusters, "OffAll");
		}

		else if(method == "individual") {
			for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();
					++it) {
				for(vector<Ptr<Node>>::iterator it_ue = it->UE.begin(); it_ue != it->UE.end();
						++it_ue) {
					Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*it_ue)->GetDevice(0));
					for(vector<Ptr<Node>>::iterator it_ap = candidate_ap.begin(); it_ap != candidate_ap.end();
							++it_ap) {
						Ptr<VlcNetDevice> Ap = DynamicCast <VlcNetDevice> ((*it_ap)->GetDevice(0));
						double dist = CalculateDistance(Ue->GetPosition(), Ap->GetPosition());
						if(dist <= da) {
							it->AP.push_back(*it_ap);
						}
					}
				}
			}
			OnOffAp(clusters, "OffAll");
		}

		else if(method == "channel-gain") {
			for(vector<Ptr<Node>>::iterator ap = candidate_ap.begin(); ap != candidate_ap.end();
										++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));
				double max_gain = 0;
				std::vector<Cluster>::iterator assigned = clusters.begin();
				for(std::vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
									++cluster) {
					if(CalculateDistance(Ap->GetPosition(), cluster->centroid) <= da) {
						double ch_gain = 0;
						for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
								++ue) {
							Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*ue)->GetDevice(0));
							double gain = EstimatedChannelGain(Ap, Ue);
							ch_gain += pow(gain, 2);
						}

						if(ch_gain > max_gain) {
							max_gain = ch_gain;
							assigned = cluster;
						}
					}
				}
				if(max_gain != 0)
					assigned->AP.push_back(*ap);
			}
		}

		else if(method == "average-channel-gain") {
			for(vector<Ptr<Node>>::iterator ap = candidate_ap.begin(); ap != candidate_ap.end();
										++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));
				double max_gain = 0;
				std::vector<Cluster>::iterator assigned = clusters.begin();
				for(std::vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
									++cluster) {
					if(CalculateDistance(Ap->GetPosition(), cluster->centroid) <= da) {
						double ch_gain = 0;
						for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
								++ue) {
							Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*ue)->GetDevice(0));
							double gain = EstimatedChannelGain(Ap, Ue);
							ch_gain += pow(gain, 2);
						}

						ch_gain /= cluster->UE.size();

						if(ch_gain > max_gain) {
							max_gain = ch_gain;
							assigned = cluster;
						}
					}
				}
				if(max_gain != 0)
					assigned->AP.push_back(*ap);
			}
		}

		else if(method == "weakest") {
			for(vector<Ptr<Node>>::iterator ap = candidate_ap.begin(); ap != candidate_ap.end();
										++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));
				double min_gain = 99999;
				std::vector<Cluster>::iterator assigned = clusters.begin();
				for(std::vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
									++cluster) {
					if(CalculateDistance(Ap->GetPosition(), cluster->centroid) <= da) {
						double ch_gain = 0;
						for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
								++ue) {
							Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*ue)->GetDevice(0));
							double gain = EstimatedChannelGain(Ap, Ue);
							ch_gain += pow(gain, 2);
						}

						if(ch_gain < min_gain && ch_gain != 0) {
							min_gain = ch_gain;
							assigned = cluster;
						}
					}
				}
				if(min_gain != 99999)
					assigned->AP.push_back(*ap);
			}
		}

		else if(method == "strongest-ue") {
			for(vector<Ptr<Node>>::iterator ap = candidate_ap.begin(); ap != candidate_ap.end();
										++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));
				double max_gain = 0;
				std::vector<Cluster>::iterator assigned = clusters.begin();
				for(std::vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
									++cluster) {
					if(CalculateDistance(Ap->GetPosition(), cluster->centroid) <= da) {
						double max_ue_gain = 0;
						for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
								++ue) {
							Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*ue)->GetDevice(0));
							double gain = EstimatedChannelGain(Ap, Ue);
							if(gain > max_ue_gain)
								max_ue_gain = gain;
						}

						if(max_ue_gain > max_gain) {
							max_gain = max_ue_gain;
							assigned = cluster;
						}
					}
				}
				if(max_gain != 0)
					assigned->AP.push_back(*ap);
			}
		}

		else {
			cout << method << endl;
			throw std::logic_error("Method not exists");
		}

		//turn off the APs that contribute 0 channel gain
		vector<vector<Ptr<Node>>::iterator> to_remove;
		bool remove = true;
		for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
				++cluster) {

			for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end()
					; ++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));
				for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
						++ue) {
					Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> ((*ue)->GetDevice(0));
					if(EstimatedChannelGain(Ap, Ue) != 0) {
						remove = false;
						break;
					}
				}
				if(remove)
					to_remove.push_back(ap);
			}

			for(vector<vector<Ptr<Node>>::iterator>::iterator it = to_remove.begin();
					it != to_remove.end(); ++it) {
				cluster->AP.erase(*it);
			}
			to_remove.clear();
		}
	}

	void OnOffAp(vector<Cluster> &clusters, string method, double index) {

		if(method == "OffAll") {
			//turn off duplicated APs

			//maybe there is a more efficient way to check duplicate APs
			vector<uint16_t> candidates;

			for(vector<Cluster>::iterator cluster = clusters.begin();
					cluster != clusters.end(); ++cluster) {

				for(vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin(); it_ap != cluster->AP.end();
						++it_ap) {

					candidates.push_back((*it_ap)->GetId());

				}
			}

			sort(candidates.begin(), candidates.end());

			uint16_t first = candidates.at(0);
			int count = 1;
			vector<uint16_t> unique;
			for(vector<uint16_t>::iterator it = candidates.begin() + 1; it != candidates.end();
					++it) {
				if(first == *it) {
					count++;
				}
				else {
					if(count == 1)
						unique.push_back(first);

					if(it == candidates.end())
						unique.push_back(*it);

					first = *it;
					count = 1;
				}
			}

			for(std::vector<Cluster>::iterator cluster = clusters.begin();
					cluster != clusters.end(); ++cluster) {

				vector<Ptr<Node>> selected_aps;

				for(vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin(); it_ap != cluster->AP.end();
						++it_ap) {

					for(vector<uint16_t>::iterator uniq = unique.begin(); uniq != unique.end();
							++uniq) {
						if((*it_ap)->GetId() == *uniq){
							selected_aps.push_back(*it_ap);
							break;
						}
					}
				}

				cluster->AP = selected_aps;
			}

			//do not remove anchoring APs
			for(std::vector<Cluster>::iterator cluster = clusters.begin();
					cluster != clusters.end(); ++cluster) {
				for(vector<Ptr<Node>>::iterator anchor = cluster->anchoring_ap.begin(); anchor != cluster->anchoring_ap.end();
						++anchor) {
					bool exist = false;
					for(vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin(); it_ap != cluster->AP.end();
							++it_ap) {
						if(*anchor == *it_ap) {
							exist = true;
							break;
						}
					}
					if(!exist)
						cluster->AP.push_back(*anchor);
				}
			}
		}

		else if(method == "OffSome") {

			map<uint16_t, double> interfering_ap;

			for(vector<Cluster>::iterator cluster = clusters.begin();
					cluster != clusters.end(); ++cluster) {
				for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin();
						ap != cluster->AP.end();) {
					Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> ((*ap)->GetDevice(0));

					double channel_gain = 0, interference_gain = 0;
					for(vector<Cluster>::iterator other_cluster = clusters.begin();
							other_cluster != clusters.end(); ++other_cluster) {
						for(vector<Ptr<Node>>::iterator ue = other_cluster->UE.begin();
								ue != other_cluster->UE.end(); ++ue) {

							Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice>
								((*ue)->GetDevice(0));

							double gain = EstimatedChannelGain(Ap, Ue);
							if(other_cluster->ID == cluster->ID)
								channel_gain += pow(gain, 2);
							else
								interference_gain += pow(gain, 2);
						}
					}
					double ind = channel_gain / interference_gain;
					if(ind > 0 && interference_gain != 0) {
						if(ind < index) {
							if(cluster->AP.size() != cluster->UE.size()) {
								ap = cluster->AP.erase(ap);
								cout << ind << " ";
								cout << "Erased!" << endl;
							}
							else
								++ap;
						}
						else
							++ap;
					}
					else
						++ap;

				}
			}
		}

		else if(method == "none") {

		}

		else if(method == "exhaustive") {
			//do it after RA
			//switch off only
		}

		else if(method == "more_exhaustive") {
			//do it after RA
			//switch on and off
		}

		else {
			cout << method << endl;
			throw std::logic_error("Method not exists");
		}
	}

	void InitUeSelection(Cluster &cluster, vector<Ptr<Node>> &candidate_ue) {
		vector<Ptr<Node>> all_ue = candidate_ue;

		uint16_t ue_c_ind = rand() % candidate_ue.size();
		Ptr<Node> ue_c = candidate_ue.at(ue_c_ind);

		cluster.UE.push_back(ue_c);
		candidate_ue.erase(candidate_ue.begin() + ue_c_ind);

		Ptr<VlcNetDevice> centroid = DynamicCast <VlcNetDevice> (ue_c->GetDevice(0));
		cluster.centroid = centroid->GetPosition();
	}

	Ptr<Node> FarthestUeSelection(vector<Cluster> &clusters, Cluster &cluster, vector<Ptr<Node>> &candidate_ue,
			string method) {

		if(method == "centroid") {

			vector<Vector> centroids;
			for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
					++cluster) {
				centroids.push_back(cluster->centroid);
			}

			double farthest_dist = 0;
			Ptr<Node> farthest_ue;
			vector<Ptr<Node>>::iterator to_erase;
			for(vector<Ptr<Node>>::iterator ue = candidate_ue.begin(); ue != candidate_ue.end();
					++ue) {
				Ptr<VlcNetDevice> Ue = DynamicCast <VlcNetDevice> ((*ue)->GetDevice(0));
				double dist = 1;
				for(vector<Vector>::iterator centroid = centroids.begin(); centroid != centroids.end();
						++centroid) {
					dist *= CalculateDistance(Ue->GetPosition(), *centroid);
				}
				if(dist > farthest_dist) {
					farthest_dist = dist;
					farthest_ue = *ue;
					to_erase = ue;
					cluster.centroid = Ue->GetPosition();
				}
			}

			cluster.UE.push_back(farthest_ue);
			candidate_ue.erase(to_erase);

			return farthest_ue;

		}

		else {
			cout << method << endl;
			throw std::logic_error("Method not exists");
		}

	}

	Vector UpdateCentroid(vector<Ptr<Node>> &devices) {
		double x, y, z = ue_height;
		double min_x, min_y, max_x, max_y;
		std::vector<Vector> pos;
		for(vector<Ptr<Node>>::iterator it = devices.begin(); it != devices.end();
				++it) {
			Ptr<VlcNetDevice> tmp = DynamicCast <VlcNetDevice> ((*it)->GetDevice(0));
			pos.push_back(tmp->GetPosition());
		}

		min_x = pos[0].x;
		min_y = pos[0].y;
		for(vector<Vector>::iterator it = pos.begin() + 1; it != pos.end(); ++it) {
			if(it->x < min_x)
				min_x = it->x;
			if(it->y < min_y)
				min_y = it->y;
		}

		max_x = pos[0].x;
		max_y = pos[0].y;
		for(vector<Vector>::iterator it = pos.begin() + 1; it != pos.end(); ++it) {
			if(it->x > max_x)
				max_x = it->x;
			if(it->y > max_y)
				max_y = it->y;
		}

		x = (max_x + min_x) / 2;
		y = (max_y + min_y) / 2;

		return Vector(x, y, z);
	}

	bool NearestSeed(Cluster &cluster, vector<Ptr<Node>> &devices, string method, uint16_t du) {
		double min_dist = sqrt(width * width + length * length);
		Ptr<Node> selected;
		vector<Ptr<Node>>::iterator to_remove;

		vector<Ptr<Node>> all_ue = devices;

		if(method == "centroid") {
			for(vector<Ptr<Node>>::iterator it = devices.begin(); it != devices.end(); ++it ) {
				Ptr<VlcNetDevice> candidate = DynamicCast <VlcNetDevice> ((*it)->GetDevice(0));
				double dist = CalculateDistance(candidate->GetPosition(),
						cluster.centroid);
				if(dist < min_dist) {
					min_dist = dist;
					selected = *it;
					to_remove = it;
				}
			}
		}

		else if(method == "edge") {

		}

		else if(method == "node") {

		}

		else {
			throw std::logic_error("Method not exists");
		}

		if(min_dist <= du) {
			for(vector<Ptr<Node>>::iterator it = all_ue.begin(); it != all_ue.end(); ++it) {
				if(*it == selected)
					selected = *it;
			}
			cluster.UE.push_back(selected);
			devices.erase(to_remove);
			return true;
		}
		else
			return false;
	}
}
