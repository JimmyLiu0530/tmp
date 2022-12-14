#ifndef VLCNETDEVICETX_H_
#define VLCNETDEVICETX_H_

#include "ns3/core-module.h"
#include "ns3/vlc-net-device.h"
#include "ns3/ptr.h"
#include "ns3/vlc-channel-model.h"
#include <cmath>
////////////////
#include <cstring>
#include "ns3/address.h"
#include "ns3/node.h"
#include "ns3/net-device.h"
#include "ns3/callback.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/ptr.h"
#include "ns3/mac48-address.h"
#include "ns3/error-model.h"
#include "ns3/queue.h"
#include "ns3/log.h"
#include <algorithm>

namespace ns3 {

class VlcChannel;

class VlcTxNetDevice: public VlcNetDevice {

public:
	VlcTxNetDevice();

	static TypeId GetTypeId(void);

	virtual ~VlcTxNetDevice();

	//adds a signal instant to the TX optical power signal
	void AddTXOpticalPowerSignal(double power);

	//returns the TX Optical power signal in the form of std::vector of doubles
	std::vector<double>& GetTXOpticalPowerSignal();

	//returns the TX power signal at instant time, time has to be lesser than the capacity of std::vector of signal
	double GetOpticalPowerSignalAtInstant(int time);

	//sets the TX power signal from a std::vector
	void SetTXOpticalPowerSignal(std::vector<double> &powerSignal);

	//reserves the capacity of signal vector
	void SetCapacity(int size);

	//returns the semiangle of the TX device
	double GetSemiangle();

	//sets the semianle of the TX device
	void SetSemiangle(double angle);

	//sets the angle of radiance of the TX device
	void SetAngleOfRadiance(double angle);

	//returns the angle of radiance of the TX device
	double GetAngleOfRadiance();

	//returns the lambertian order of the TX device
	double GetLambertianOrder();

	//computes and then sets the lambertian order of the TX device
	void SetLambertainOrder();

	//returns the gain of the TX device
	double GetTXGain();
	//computes and sets the gain of the TX device
	void SetTXGain();

	//adds a signal component to the signal vector
	void AddSignal(double signal);

	//returns the signal vector of the TX device
	std::vector<double>& GetSignal();

	//returns the magnitude of the signal at a particular instant
	double GetSignalAtInstant(int time);

	//sets the signal vector of the TX device
	void SetSignal(std::vector<double> &signal);

	//sets the bias voltage of the TX device
	void SetBias(double bias);

	//returns the bias voltage of the TX device
	double GetBias();

	//calculates the optical power signal after biasing it by m_bias
	void BoostSignal();

	void SetTxPowerMax(double pMax);

	//returns the Maximum TX power that can be transmitted
	double GetTXPowerMax();

	//returns the average of TX Power signal
	double GetAveragePowerSignalPower();

	//returns the average of the TX signal
	double GetAverageSignal();

	double GetAveragePower();
	void SetAveragePower(double val);

	void EnqueueDataPacket(Ptr<Packet> p);

	void TransmitComplete(double t);

	void AttachChannel(Ptr<VlcChannel> channel);

	void SetPrecodingVector(std::map<uint16_t, double> precoding_term);
	std::map<uint16_t, double> GetPrecodingVector();

	void SetVectorPower(uint16_t rx_id, double power);
	double GetVectorPower(uint16_t rx_id);

	void SetDataRateInbps(uint16_t rx_id, double rate);
	double GetDataRateInbps(uint16_t rx_id);

	bool IsPowerAllocated();

	double GetTotalTransmitPower();

	void SetTXRateMax(double RMax);
	double GetTXRateMax();

	double GetTotalPowerConsumption(); //including the precoding cost

	void Clear();

	void SetMinRequiredPower(double power);
	double GetMinRequiredPower();

	double GetPrecodingTerm(uint16_t rx_id);

private:

	std::vector<double> m_TXOpticalPower;

	std::vector<double> m_signal; 
	double m_TMAX;
	double m_RMAX;
	double m_semiangle;
	double m_angleOfRadiance;
	double m_lOrder;
	double m_TXGain;
	double m_bias;
	Ptr<VlcChannel> m_channel;

	std::map<uint16_t, double> m_precoding_term;
	//<rx_id, precoding term>
	std::map<uint16_t, double> vector_power;

	std::map<uint16_t, double> m_data_rate;

	double min_rq_power;
};

} /* namespace vlc */

#endif /* VLCNETDEVICETX_H_ */
