#include "../thesis/resource-alloc.h"

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "../thesis/clustering.h"
#include "../thesis/parameter-config.h"
#include "../thesis/device-config.h"
#include "../thesis/matrix.h"
#include "ns3/core-module.h"
#include "../thesis/math.c"


using namespace std;

namespace ns3 {
	map<VlcErrorModel::ModScheme, uint16_t> mod_order;

	vector< map<uint16_t, map<uint16_t, double> > > Precoding(vector<Cluster> &clusters) {
		vector< map<uint16_t, map<uint16_t, double> > > precoding_matrices;

		for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
				++cluster) {
			if(cluster->UE.size() >= 2 && !cluster->AP.empty()) {
				//VT
				////map<uint16_t, map<uint16_t, double>>   ----> I tried to use map instead of vector for matrix calculation, but failed....QQ
				vector<vector<double>> channel_matrix, precoding_matrix;
				channel_matrix = CreateChannelMatrix(*cluster);
/*
				cout << "channel matrix" << endl;
				PrintOutMatrix(channel_matrix);
*/
				precoding_matrix = PseudoInverseMatrix(channel_matrix);

				map<uint16_t, map<uint16_t, double> > tmp;

				vector<Ptr<Node>>::iterator ap = cluster->AP.begin();
				vector<vector<double>>::iterator row = precoding_matrix.begin();

				for(;ap != cluster->AP.end() && row != precoding_matrix.end();
						++ap, ++row) {
					vector<Ptr<Node>>::iterator ue = cluster->UE.begin();
					vector<double>::iterator col = row->begin();
					for(;ue != cluster->UE.end() && col != row->end();
							++ue, ++col) {
						uint16_t ap_id = (*ap)->GetId(),
								ue_id = (*ue)->GetId();
						tmp[ap_id][ue_id] = *col;
					}
				}
				precoding_matrices.push_back(tmp);

				cluster->precoding_matrix = tmp;
/*
				cout << "precoding matrix" << endl;
				PrintOutMatrix(precoding_matrix);
*/
				//set precoding vector for each tx
				vector<vector<double>>::iterator v = precoding_matrix.begin();
				vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin();

				for(; it_ap != cluster->AP.end() && v != precoding_matrix.end();
						++it_ap, ++v) {
						Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
											(*it_ap)->GetDevice(0));

						map<uint16_t, double> precoding_vector;

						vector<Ptr<Node>>::iterator it_ue = cluster->UE.begin();
						vector<double>::iterator precoding_term = v->begin();

						for(; it_ue != cluster->UE.end() && precoding_term != v->end()
								; ++it_ue, ++precoding_term) {
							uint16_t ue_id = (*it_ue)->GetId();
							precoding_vector.insert(pair<uint16_t, double>(ue_id , *precoding_term));
						}

						Ap->SetPrecodingVector(precoding_vector);
						precoding_vector.clear();

				}

				//*****testing only*****************
				////map<uint16_t, map<uint16_t, double>>
				/*vector<vector<double>> identity;
				identity = Dot(channel_matrix, precoding_matrix);
				cout << "-------------------" << endl;
				cout << "identity" << endl;
				PrintOutMatrix(identity);
				cout << "-------------------" << endl;*/
			}
		}

		return precoding_matrices;

	}

	double ResourceAllocation(vector<Cluster> &clusters, uint16_t interf_iteration_num,
			string vt_ap_iteration_method, string interf_calc) {

		//clear allocated resource first
		for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
			for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end();
					++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
						(*ap)->GetDevice(0));
				Ap->Clear();
			}
		}

		Precoding(clusters);

		mod_order.insert(pair<VlcErrorModel::ModScheme, uint16_t>(VlcErrorModel::BPSK, 1));
		mod_order.insert(pair<VlcErrorModel::ModScheme, uint16_t>(VlcErrorModel::QAM4, 2));
		mod_order.insert(pair<VlcErrorModel::ModScheme, uint16_t>(VlcErrorModel::QAM16, 3));
		mod_order.insert(pair<VlcErrorModel::ModScheme, uint16_t>(VlcErrorModel::QAM64, 4));
		mod_order.insert(pair<VlcErrorModel::ModScheme, uint16_t>(VlcErrorModel::QAM256, 5));

		//sort the clusters from the smallest to the largest
		sort(clusters.begin(), clusters.end(), CompareClusterSize);

		double total_ee;

		for(uint16_t i = 0; i < interf_iteration_num; i++) {
			total_ee = 0;
			//for each cluster
			int loop_num = 1;
			for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
				if(!cluster->AP.empty()) {
					//VT
					if(cluster->UE.size() >= 2) {

						Knapsack strategy = VT_RA_Algorithm(clusters,
								*cluster, pow(Pmax, 2) * 2 * M_PI,
								vt_ap_iteration_method, interf_calc);

						total_ee += strategy.EE;

						//Allocate power and modulation mode
						ModOrderSelectionAndPowerAlloc(*cluster, strategy.mod_order, strategy.power);

					}

					//CT
					else if(cluster->UE.size() == 1) {

						Knapsack strategy = CT_RA_Algorithm(clusters, *cluster, pow(Pmax, 2) * 2 * M_PI,
								interf_calc);

						total_ee += strategy.EE;
					}
				}

				else {
					for(vector<Ptr<Node>>::iterator ue = cluster->UE.begin(); ue != cluster->UE.end();
							++ue) {
						Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
								(*ue)->GetDevice(0));
						Ue->Idle();
					}
				}
			}
		}

		return total_ee / clusters.size();

	}

	double OnOffAP(vector<Cluster> &clusters, double ee, uint16_t interf_iteration_num,
			string vt_ap_iteration_method, string interf_calc, string method) {

		double current_ee = ee, total_ee = ee;
		uint16_t loop_num = 0;

		map<uint16_t, double> interfering_ap;

		for(vector<Cluster>::iterator cluster = clusters.begin();
				cluster != clusters.end(); ++cluster) {
			for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin();
					ap != cluster->AP.end(); ++ap) {
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
				double index = channel_gain / interference_gain;
				if(index > 0 && interference_gain != 0)
					interfering_ap[(*ap)->GetId()] = index;
			}
		}

		//sort them ascendingly
		vector<double> sorting;
		for(map<uint16_t, double>::iterator it = interfering_ap.begin();
				it != interfering_ap.end(); ++it) {
			sorting.push_back(it->second);
		}

		sort(sorting.begin(), sorting.end());

		cout << "Interfering index...>" << endl;

		vector<uint16_t> on_off_ap;
		for(vector<double>::iterator index = sorting.begin(); index !=sorting.end();
				++index) {
			cout << *index << " ";
			for(map<uint16_t, double>::iterator it = interfering_ap.begin();
					it != interfering_ap.end(); ++it) {
				if(*index == it->second)
					on_off_ap.push_back(it->first);
			}
		}
		cout << endl;

		if(method == "exhaustive") {

			vector<Cluster> optimal_clusters;

			//switch off for each
			do {
				loop_num++;

				bool reallocate;

				total_ee = current_ee;

				optimal_clusters = clusters;

				for(vector<uint16_t>::iterator it = on_off_ap.begin(); it != on_off_ap.end();) {

					cout << *it << endl;
					reallocate = false;

					vector<Ptr<Node>>::iterator to_remove;
					vector<Cluster>::iterator cluster = clusters.begin();
					bool find;

					for(; cluster != clusters.end(); ++cluster) {
						find = false;
						for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end();
								++ap) {
							if(*it == (*ap)->GetId()) {
								to_remove = ap;
								find = true;
								break;
							}
						}
						if(find)
							break;
					}

					if(!find)
						throw std::logic_error("AP not found");

					Ptr<Node> the_ap = *to_remove;
					if(cluster->AP.size() != cluster->UE.size()) {
						cluster->AP.erase(to_remove);
						vector<Cluster> tmp = clusters;
						double new_ee = ResourceAllocation(tmp, interf_iteration_num,
								vt_ap_iteration_method, interf_calc);

						if(new_ee < current_ee) {
							cluster->AP.push_back(the_ap);
							reallocate = true;
						}
						else {
							current_ee = new_ee;
							reallocate = false;
						}
					}

					//if it is switched off, it doesn't need to be iterate in the next round
					if(!reallocate)
						on_off_ap.erase(it);
					else
						++it;
				}

				cout << "AP set" << endl;
				for(std::vector<Cluster>::iterator cluster = clusters.begin();
						cluster != clusters.end(); ++cluster) {
					for(std::vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin();
							it_ap != cluster->AP.end(); ++it_ap) {
						cout << (*it_ap)->GetId() << " ";
					}
					cout << endl;
				}

				cout << current_ee << endl;

			} while(current_ee - total_ee > 1e-8 && loop_num < on_off_ap.size());

			clusters = optimal_clusters;
			ResourceAllocation(clusters, interf_iteration_num,
					vt_ap_iteration_method, interf_calc);

		}
		else if(method == "more_exhaustive") {

			vector<Cluster> original_clusters = clusters, optimal_clusters;

			map<uint16_t, bool> status;

			for(vector<uint16_t>::iterator s = on_off_ap.begin(); s != on_off_ap.end(); ++s)
				status[*s] = true;

			//change status of each AP
			do {
				loop_num++;

				optimal_clusters = clusters;

				total_ee = current_ee;

				for(vector<uint16_t>::iterator it = on_off_ap.begin(); it != on_off_ap.end();
						++it) {

					cout << *it << endl;

					vector<Ptr<Node>>::iterator to_remove;
					bool find;

					//if AP is switched on
					if(status[*it]) {
						vector<Cluster>::iterator cluster = clusters.begin();
						for(; cluster != clusters.end(); ++cluster) {
							find = false;
							for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end();
									++ap) {

								if(*it == (*ap)->GetId()) {
									to_remove = ap;
									find = true;
									break;
								}
							}
							if(find)
								break;
						}
						if(!find) {
							cout << status[*it] << endl;
							throw std::logic_error("AP not found");
						}

						Ptr<Node> the_ap = *to_remove;
						if(cluster->AP.size() != cluster->UE.size()) {
							cluster->AP.erase(to_remove);
							vector<Cluster> tmp = clusters;
							double new_ee = ResourceAllocation(tmp, interf_iteration_num,
									vt_ap_iteration_method, interf_calc);

							if(new_ee < current_ee) {
								cluster->AP.push_back(the_ap);
							}
							else {
								current_ee = new_ee;
								status[*it] = false;
							}
						}

					}
					//if AP is switched off
					else {
						uint16_t id;

						for(vector<Cluster>::iterator cluster = original_clusters.begin(); cluster != original_clusters.end(); ++cluster) {
							find = false;
							for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end();
									++ap) {

								if(*it == (*ap)->GetId()) {
									to_remove = ap;
									find = true;
									id = cluster->ID;
									break;
								}
							}
							if(find)
								break;
						}
						if(!find) {
							cout << status[*it] << endl;
							throw std::logic_error("AP not found");
						}

						clusters[id].AP.push_back(*to_remove);
						vector<Cluster> tmp = clusters;
						double new_ee = ResourceAllocation(tmp, interf_iteration_num,
								vt_ap_iteration_method, interf_calc);

						if(new_ee < current_ee) {
							clusters.at(id).AP.pop_back();
						}
						else {
							current_ee = new_ee;
							status[*it] = true;
						}
					}

				}

				cout << "AP set" << endl;
				for(std::vector<Cluster>::iterator cluster = clusters.begin();
						cluster != clusters.end(); ++cluster) {
					for(std::vector<Ptr<Node>>::iterator it_ap = cluster->AP.begin();
							it_ap != cluster->AP.end(); ++it_ap) {
						cout << (*it_ap)->GetId() << " ";
					}
					cout << endl;
				}

				cout << current_ee << endl;

			} while(current_ee - total_ee > 1e-8 && loop_num < on_off_ap.size());

			clusters = optimal_clusters;
			ResourceAllocation(clusters, interf_iteration_num,
					vt_ap_iteration_method, interf_calc);

		}

		return total_ee;

	}

	Knapsack VT_RA_Algorithm(vector<Cluster> clusters, Cluster cluster, double capacity,
			string vt_ap_iteration_method, string interf_calc) {

		vector<Ptr<Node>>::iterator ap_it = cluster.AP.begin();
		Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
				(*ap_it)->GetDevice(0));
		double Rmax = Ap->GetTXRateMax();

		map<double, map<int, Knapsack>> knapsack;

		Knapsack strategy;
		double max_EE = -1.0;

		if(cluster.AP.empty()) {

			strategy.EE = 0;
			for(vector<Ptr<Node>>::iterator ue = cluster.UE.begin();
					ue != cluster.UE.end(); ++ue) {

				uint16_t ue_id = (*ue)->GetId();

				strategy.power[ue_id] = 0;
			}

			return strategy;
		}

		//knapsack problem
		uint16_t ap_id;
		map<uint16_t, double> precoding_term;

		if(vt_ap_iteration_method == "one") {

			ap_id = cluster.AP.at(rand() % cluster.AP.size())->GetId();

			for(map<uint16_t, double>::iterator term = cluster.precoding_matrix[ap_id].begin();
					term != cluster.precoding_matrix[ap_id].end(); ++term) {
				uint16_t ue_id = term->first;
				precoding_term[ue_id] = pow(term->second, 2);
			}

		}

		else if(vt_ap_iteration_method == "average") {

			capacity *= cluster.AP.size();

			for(map<uint16_t, map<uint16_t, double>>:: iterator vec = cluster.precoding_matrix.begin();
					vec != cluster.precoding_matrix.end(); ++vec) {

				for(map<uint16_t, double>::iterator term = vec->second.begin();
						term != vec->second.end(); ++term) {
					uint16_t ue_id = term->first;
					precoding_term[ue_id] = 0;
				}
				break;
			}

			for(map<uint16_t, map<uint16_t, double>>:: iterator vec = cluster.precoding_matrix.begin();
					vec != cluster.precoding_matrix.end(); ++vec) {

				for(map<uint16_t, double>::iterator term = vec->second.begin();
						term != vec->second.end(); ++term) {
					uint16_t ue_id = term->first;
					precoding_term[ue_id] += pow(term->second, 2);
				}
			}
		}

		for(uint16_t j = 1; j <= J; j++) {

			uint16_t last_ue = -1;
			double max_power = capacity / J * j;

			knapsack[max_power][-1].EE = 0;

			for(vector<Ptr<Node>>::iterator ue = cluster.UE.begin();
					ue != cluster.UE.end(); ++ue) {

				uint16_t ue_id = (*ue)->GetId();

				knapsack[max_power][-1].power[ue_id] = 0;
			}

			//consider each UE (item) to be put into the knapsack
			for(vector<Ptr<Node>>::iterator ue = cluster.UE.begin();
					ue != cluster.UE.end(); ++ue) {

				uint16_t ue_id = (*ue)->GetId();

				double current_max_EE = knapsack[max_power][last_ue].EE;

				knapsack[max_power][ue_id] = knapsack[max_power][last_ue];

				//for each modulation order (choice), pick the order that maximizes current EE
				for(map<VlcErrorModel::ModScheme, uint16_t>::iterator mod = mod_order.begin();
						mod != mod_order.end(); ++mod) {

					VlcErrorModel::ModScheme alloc_order = mod->first;
					uint16_t order = mod->second;

					//then calculate minimum required power
					double interference = EstimatedInterference(clusters,
							cluster, *ue, interf_calc);
					//cout << interference << endl;
					//interference = 0;
					double power_rq = MinPowerRequired(order, interference);

					//then decide which modulation order should the UE choose
					//put or not put
					if(precoding_term[ue_id] * power_rq <= max_power) {

						double remain_capa = max_power -
								precoding_term[ue_id] * power_rq;

						for(uint16_t i = 1; i <= j; i++) {
							if(remain_capa < (capacity / J * i)) {
								remain_capa = capacity / J * (i - 1);
								break;
							}
						}

						double ee = 0;

						//calculate EE after put this UE in
						double throughput = EstimatedThroughput(order);
						double consumed_power =
								precoding_term[ue_id] * power_rq;

						if(!knapsack[remain_capa][last_ue].mod_order.empty()) {

							for(map<uint16_t, VlcErrorModel::ModScheme>::iterator m =
									knapsack[remain_capa][last_ue].mod_order.begin();
									m != knapsack[remain_capa][last_ue].mod_order.end();
									m++) {

								throughput += EstimatedThroughput(mod_order[m->second]);

							}

							for(map<uint16_t, double>::iterator p =
									knapsack[remain_capa][last_ue].power.begin();
									p != knapsack[remain_capa][last_ue].power.end();
									p++) {

								uint16_t ue_i = p->first;
								double power = p->second;

								consumed_power += precoding_term[ue_id] * power;
							}
							if(consumed_power != 0)
								ee = throughput / consumed_power;
						}

						if(ee >= current_max_EE) {

							knapsack[max_power][ue_id] = knapsack[remain_capa][last_ue];
							knapsack[max_power][ue_id].mod_order[ue_id] = alloc_order;
							knapsack[max_power][ue_id].power[ue_id] = power_rq;
							knapsack[max_power][ue_id].EE = ee;
							current_max_EE = ee;

						}
					}
				}
				//cout << knapsack[max_power][ue_id].EE << " ";
				last_ue = ue_id;

			}
		}


			//Print out what the fuck is in the fucking knapsack
/*
							for(map<double, map<int, Knapsack>>::iterator sol = knapsack.begin();
									sol != knapsack.end(); ++sol) {
								for(map<int, Knapsack>::iterator it = sol->second.begin();
										it != sol->second.end(); ++it) {
									cout << it->second.EE << " ";

								}
								cout << endl;

							}
*/

		//Checking max power constraint & backhaul constraint
		for(map<double, map<int, Knapsack>>::iterator k = knapsack.begin();
				k != knapsack.end(); ++k) {
			for(map<int, Knapsack>::iterator sol = k->second.begin();
					sol != k->second.end(); ++sol) {

				double ee = sol->second.EE;

				if(BackhaulConstraintSatisfied(sol->second, Rmax)
						&& PowerConstraintSatisfied(sol->second, cluster)) {
					if(ee > max_EE) {
						max_EE = ee;
						strategy = sol->second;
					}
					else if(ee == max_EE) {
						//compare active number of UEs first
						if(sol->second.mod_order.size() > strategy.mod_order.size()) {
							max_EE = ee;
							strategy = sol->second;
						}
						else if (sol->second.mod_order.size() == strategy.mod_order.size()) {
							//compare power consumption
							double candidate_total_power = 0, strategy_total_power = 0;

							for(map<uint16_t, double>::iterator p = sol->second.power.begin();
									p != sol->second.power.end(); ++p) {
								double power = p->second;
								uint16_t ue_id = p->first;
								candidate_total_power += power * precoding_term[ue_id];
							}

							for(map<uint16_t, double>::iterator p = strategy.power.begin();
									p != strategy.power.end(); ++p) {
								double power = p->second;
								uint16_t ue_id = p->first;
								strategy_total_power += power * precoding_term[ue_id];
							}

							if(candidate_total_power < strategy_total_power) {
								max_EE = ee;
								strategy = sol->second;
							}
						}
					}
				}
			}
		}


		//if the RA results that only one UE is active, it has to switch to CT from VT
		if(strategy.mod_order.size() == 1) {
			Cluster new_cluster = cluster;
			new_cluster.UE.clear();
			for(vector<Ptr<Node>>::iterator ue = cluster.UE.begin(); ue != cluster.UE.end();
					++ue) {
				if((*ue)->GetId() == strategy.mod_order.begin()->first) {
					new_cluster.UE.push_back(*ue);
					break;
				}
			}

			//clear allocated resource first
			for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
				for(vector<Ptr<Node>>::iterator ap = cluster->AP.begin(); ap != cluster->AP.end();
						++ap) {
					Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
							(*ap)->GetDevice(0));
					Ap->Clear();
				}
			}

			return CT_RA_Algorithm(clusters, new_cluster, capacity, interf_calc);
		}

		return strategy;
	}

	Knapsack CT_RA_Algorithm(vector<Cluster> clusters, Cluster cluster, double capacity,
			string interf_calc) {
		vector<Ptr<Node>>::iterator ap_it = cluster.AP.begin();
		Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
				(*ap_it)->GetDevice(0));
		double Rmax = Ap->GetTXRateMax();

		Ptr<Node> UE = cluster.UE.at(0);
		Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
				UE->GetDevice(0));
		uint16_t ue_id = UE->GetId();

		vector<Knapsack> candidates;

		for(map<VlcErrorModel::ModScheme, uint16_t>::iterator mod = mod_order.begin();
				mod != mod_order.end(); ++mod) {

			uint16_t order = mod->second;

			//check if it violates power constraint and backhaul constraint
			if(EstimatedThroughput(order) <= Rmax) {
				//calculate required power
				double interference = EstimatedInterference(clusters,
						cluster, cluster.UE.at(0), interf_calc);
				double power_rq = MinPowerRequired(order, interference);
				//calculate EE
				double throughput = EstimatedThroughput(order);
				double ee = throughput / power_rq;

				Knapsack knapsack;
				knapsack.EE = ee;
				knapsack.mod_order[ue_id] = mod->first;
				knapsack.power[ue_id] = power_rq;
				candidates.push_back(knapsack);
			}

		}

		//sort the candidates (modulation order)
		sort(candidates.begin(), candidates.end(), CompareKnapsack);

		Knapsack strategy;
		bool allocated = false;

		vector<pair<uint16_t, double>> AP_order;

		for(vector<Ptr<Node>>::iterator ap_it = cluster.AP.begin(); ap_it != cluster.AP.end();
				++ap_it) {
			Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
					(*ap_it)->GetDevice(0));
			double ch_gain = EstimatedChannelGain(Ap, Ue);
			AP_order.push_back(make_pair((*ap_it)->GetId(), ch_gain));
		}

		/*allocate power from the nearest AP*/
		//sort APs from the nearest to the farthest to the UE
		sort(AP_order.begin(), AP_order.end(), CompareAP);

		map<uint16_t, double> power_alloc;

		if(!candidates.empty()) {

			bool fulfill = false;
			double power_rq = 0;

			for(vector<Knapsack>::iterator candidate = candidates.begin();
					candidate != candidates.end(); ++candidate) {

				power_rq = candidate->power.begin()->second;
				double total_power = 0;
				fulfill = false;

				for(vector<pair<uint16_t, double>>::iterator it = AP_order.begin();
						it != AP_order.end(); ++it) {

					uint16_t ap_id = it->first;
					double ch_gain = it->second;

					if(fulfill) {
						power_alloc[ap_id] = 0;
					}
					else {
						double max_power = 2 * M_PI * pow(Pmax, 2);

						if(ch_gain == 0) {
							power_alloc[ap_id] = 0;
						}
						else {
							if(pow(total_power + sqrt(max_power) * ch_gain, 2) < power_rq) {
								power_alloc[ap_id] = max_power;
								total_power += sqrt(max_power) * ch_gain;
							}
							else {
								power_alloc[ap_id] = pow((sqrt(power_rq) - total_power) / ch_gain, 2);
								fulfill = true;
								break;
							}
						}
					}
				}

				if(fulfill) {
					strategy = *candidate;
					break;
				}

				power_alloc.clear();

			}

			if(fulfill) {
				double consumed_power = 0;
				for(map<uint16_t, double>::iterator it = power_alloc.begin();
						it != power_alloc.end(); ++it) {

					Ptr<VlcTxNetDevice> Ap;

					for(vector<Ptr<Node>>::iterator ap = cluster.AP.begin();
							ap != cluster.AP.end(); ++ap) {
						if((*ap)->GetId() == it->first) {
							Ap = DynamicCast <VlcTxNetDevice> (
									(*ap)->GetDevice(0));
							break;
						}
					}

					double power = it->second;
					consumed_power += power;
					Ap->SetVectorPower(ue_id, power);
					Ap->SetMinRequiredPower(power_rq);

					VlcErrorModel::ModScheme order = strategy.mod_order[ue_id];
					double rate = EstimatedThroughput(mod_order[order]);
					Ap->SetDataRateInbps(ue_id, rate);

					//modulation order selection
					Ue->SetScheme(order);
				}

				Ue->Active();

				strategy.EE *= power_rq / consumed_power;

				return strategy;
			}

			else {
				for(vector<Ptr<Node>>::iterator ap = cluster.AP.begin();
						ap != cluster.AP.end(); ++ap) {
					Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
							(*ap)->GetDevice(0));
					Ap->SetVectorPower(ue_id, 0);
				}
				Ue->Idle();
			}

		}
		else {
			for(vector<Ptr<Node>>::iterator ap = cluster.AP.begin();
					ap != cluster.AP.end(); ++ap) {
				Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
						(*ap)->GetDevice(0));
				Ap->SetVectorPower(ue_id, 0);
			}
			Ue->Idle();
		}

		Knapsack knapsack;
		knapsack.EE = 0;
		knapsack.power[ue_id] = 0;

		return knapsack;

	}

	bool BackhaulConstraintSatisfied(Knapsack knapsack, double Rmax) {
		double rate = 0;
		for(map<uint16_t, VlcErrorModel::ModScheme>::iterator m = knapsack.mod_order.begin();
				m != knapsack.mod_order.end(); ++m) {
			uint16_t order = mod_order[m->second];
			rate += EstimatedThroughput(order);
		}

		if(rate > Rmax)
			return false;
		else
			return true;
	}

	bool PowerConstraintSatisfied(Knapsack knapsack, Cluster cluster) {

		for(map<uint16_t, map<uint16_t, double> >:: iterator vec = cluster.precoding_matrix.begin();
				vec != cluster.precoding_matrix.end(); ++vec) {

			double total_power = 0;

			for(map<uint16_t, double>::iterator m = knapsack.power.begin();
					m != knapsack.power.end(); ++m) {
				double power = m->second;
				uint16_t ue_id = m->first;
				total_power += power * pow(vec->second[ue_id], 2);
			}

			if(total_power > 2 * M_PI * pow(Pmax, 2))
				return false;
		}

		return true;

	}

	double EstimatedThroughput(double order) {
		return order * bw / 2;
	}

	double MinPowerRequired(double order, double interference) {
		/*cout << "estimated" << endl;
		cout << interference << endl;*/

		//calculate required SINR
		double sinr_rq;
		sinr_rq = (order * pow(2, order) * ber_target)
				/ (pow(2, order) - 1);
		sinr_rq = erfinv(1 - sinr_rq);
		sinr_rq *= sinr_rq;
		sinr_rq *= (2 * (pow(2, 2 * order) - 1) / 3);
		//cout << "estimated" << endl;
		//cout << sinr_rq << endl;

		//then calculate minimum required power
		double power_rq;
		const double noise = noise_spectral_density * bw;
		power_rq = sinr_rq * (noise + interference) / (pow(res, 2) / 2);
		//cout << power_rq << endl;

/*		double errorRate = sqrt((3 * sinr_rq)/(2 * (pow(2, 2 * order) - 1)));
		errorRate = erfc(errorRate);
		errorRate *= (pow(2, order) - 1) / (order * pow(2, order));

		cout << errorRate << endl;*/

		return power_rq;
	}

	double EstimatedInterference(vector<Cluster> &clusters, Cluster cluster, Ptr<Node> ue,
			string method) {

		double interference_total = 0;

		for(vector<Cluster>::iterator c = clusters.begin(); c != clusters.end(); ++c) {

			Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
									(ue)->GetDevice(0));
			uint16_t i = c->ID;

			if(i != cluster.ID) {

				for(vector<Ptr<Node>>::iterator ap = clusters.at(i).AP.begin();
						ap != clusters.at(i).AP.end(); ++ap) {

					Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
							(*ap)->GetDevice(0));

					double ch_gain = EstimatedChannelGain(Ap, Ue);

					double interf_power;
					if(clusters.at(i).UE.size() >= 2)
						interf_power = MaxTransmitPower(clusters.at(i).precoding_matrix, method);
					else
						interf_power = 2 * M_PI * pow(Pmax, 2);
					if(Ap->IsPowerAllocated()) {
						interf_power = Ap->GetTotalTransmitPower();
					}

					interference_total += interf_power * pow(ch_gain * Ue->GetResponsivity(), 2) / 2;
				}
			}
		}

		return interference_total;
	}

	double MaxTransmitPower(map<uint16_t, map<uint16_t, double> > precoding_matrix,
			string method) {

		if(method == "worst-case")
			return 2 * M_PI * pow(Pmax, 2);

		else if(method == "approximate") {

			double max_term = -1;

			//VT

			for(map<uint16_t, map<uint16_t, double> >::iterator vec = precoding_matrix.begin();
					vec != precoding_matrix.end(); ++vec) {

				double smallest = pow(vec->second.begin()->second, 2);

				for(map<uint16_t, double>::iterator term = vec->second.begin();
						term != vec->second.end(); ++term) {
					double precoding_term = pow(term->second, 2);
					if(precoding_term < smallest)
						smallest = precoding_term;
				}

				if(smallest > max_term && smallest != 0)
					max_term = smallest;
			}

			if(max_term == -1)
				return 2 * M_PI * pow(Pmax, 2);
			else
				return 2 * M_PI * pow(Pmax, 2) / max_term;
		}

		else {
			cout << method << endl;
			throw std::logic_error("Method not exists");
		}


	}

	void ModOrderSelectionAndPowerAlloc(Cluster cluster,
			map<uint16_t, VlcErrorModel::ModScheme> m_order,
			map<uint16_t, double> power) {

		if(cluster.UE.size() >= 2) {
			for(vector<Ptr<Node>>::iterator ap = cluster.AP.begin(); ap != cluster.AP.end();
					++ap) {
				for(vector<Ptr<Node>>::iterator ue = cluster.UE.begin(); ue != cluster.UE.end();
						++ue) {
					Ptr<VlcTxNetDevice> Ap = DynamicCast <VlcTxNetDevice> (
							(*ap)->GetDevice(0));
					Ptr<VlcRxNetDevice> Ue = DynamicCast <VlcRxNetDevice> (
							(*ue)->GetDevice(0));

					uint16_t ue_id = (*ue)->GetId();

					bool find = false;

					for(map<uint16_t, double>::iterator it = power.begin();
							it != power.end(); ++it) {
						if(it->first == ue_id && it->second != 0) {
							find = true;
							break;
						}
					}

					if(find) {
						Ap->SetVectorPower(ue_id, power[ue_id]);

						VlcErrorModel::ModScheme order = m_order[ue_id];
						double rate = EstimatedThroughput(mod_order[order]);
						Ap->SetDataRateInbps(ue_id, rate);
						//cout << ue_id << ":" << rate <<  endl;
						//cout << (*ap)->GetId() << endl;
						//cout << "********************" << endl;

						//modulation order selection
						Ue->SetScheme(order);
						Ue->Active();
					}

					else {
						Ue->Idle();
					}

				}
			}
		}
		else {


		}

	}

	/*double ErfcInv(double x) {
			x = 1 - x;

		   double tt1, tt2, lnx, sgn;
		   sgn = (x < 0) ? -1.0f : 1.0f;

		   x = (1 - x)*(1 + x);        // x = 1 - x*x;
		   lnx = log(x);

		   tt1 = 2/(M_PI*0.15449436008930206298828125) + 0.5 * lnx;
		   tt2 = 1/(0.15449436008930206298828125) * lnx;

		   return(sgn*sqrt(-tt1 + sqrt(tt1*tt1 - tt2))*3);

	}*/

	bool CompareKnapsack(const Knapsack k1, const Knapsack k2) {

		if(k1.EE > k2.EE || (k1.EE == k2.EE && k1.power.begin()->second < k2.power.begin()->second))
				return true;
		else
				return false;
	}

	bool CompareAP(const pair<uint16_t, double> &a1, const pair<uint16_t, double> &a2) {

		return a1.second > a2.second;

	}

	bool CompareClusterSize(const Cluster c1, const Cluster c2) {
		if(c1.UE.size() != c2.UE.size())
			return c1.UE.size() < c2.UE.size();
		else if(c1.UE.size() == c2.UE.size())
			return c1.AP.size() < c2.AP.size();
	}

}
