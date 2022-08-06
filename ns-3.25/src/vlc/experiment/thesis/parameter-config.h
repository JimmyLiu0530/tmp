/*
 * parameter-config.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_PARAMETER_CONFIG_H_
#define SCRATCH_PARAMETER_CONFIG_H_

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"

using namespace std;

namespace ns3{

	constexpr uint16_t ap_num = 64;
	//room size
	constexpr double length = 15.0, width = 15.0, height = 3.0;
	//device position
	constexpr double ue_height = 0.85, ap_height = 2.5;
	//resource
	constexpr double Pmax = 10;
	//AP parameter
	constexpr double bias = 0.5, semi_angle = 70, azimuth = 0 ,
			elevation = 180.0, gain = 70;
	//UE parameter
	constexpr double filter_gain = 1, refractive_index = 1.5,
			concentration_gain = 0,
			rxgain = 0, beta = 1;
	constexpr double Band_factor_Noise_Signal = (10.0);
	constexpr double res = 0.7; //responsivity
	constexpr double noise_spectral_density = 1e-22;

	//channel parameter
	constexpr double wl_low = 380, wl_high = 780, delay = 0;
	constexpr double bw = 50 * 1e6, A = 1e-4, temp = 295; // A: photodetector area

	struct Cluster {
		vector<Ptr<Node>> AP;
		vector<Ptr<Node>> anchoring_ap;
		vector<Ptr<Node>> UE;
		Vector centroid;
		uint16_t ID;
		map<uint16_t, map<uint16_t, double> > precoding_matrix;
	};

	//clustering paramters
	//constexpr uint16_t du = 1, da = 5;

	constexpr uint16_t M = 5;

	//DP
	constexpr uint16_t J = 20; //to discretize Pmax

	constexpr double ber_target = 1e-5;

}



#endif /* SCRATCH_PARAMETER_CONFIG_H_ */
