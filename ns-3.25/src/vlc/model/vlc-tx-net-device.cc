#include "ns3/vlc-tx-net-device.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("vlcNetDeviceTX");
NS_OBJECT_ENSURE_REGISTERED (VlcTxNetDevice);

ns3::TypeId VlcTxNetDevice::GetTypeId(void)	// returns meta-information about VLC_ErrorModel class
		{ 	// including parent class, group name, constructor, and attributes

	static ns3::TypeId tid = ns3::TypeId("VlcTxNetDevice").SetParent<
			VlcNetDevice>().AddConstructor<VlcTxNetDevice>().AddAttribute(
			"SemiAngle", "Semi angle for the TX device", DoubleValue(0),
			MakeDoubleAccessor(&VlcTxNetDevice::m_semiangle),
			MakeDoubleChecker<double>()).AddAttribute("AngleOfRadiance",
			"Angle of radiance for the TX device", DoubleValue(0),
			MakeDoubleAccessor(&VlcTxNetDevice::m_angleOfRadiance),
			MakeDoubleChecker<double>()).AddAttribute("LambertianOrder",
			"Lambertian Order for the TX device", DoubleValue(0),
			MakeDoubleAccessor(&VlcTxNetDevice::m_lOrder),
			MakeDoubleChecker<double>()).AddAttribute("Gain",
			"TX gain for the TX device", DoubleValue(0),
			MakeDoubleAccessor(&VlcTxNetDevice::m_TXGain),
			MakeDoubleChecker<double>()).AddAttribute("Bias",
			"Biasing voltage for the TX device", DoubleValue(0),
			MakeDoubleAccessor(&VlcTxNetDevice::m_bias),
			MakeDoubleChecker<double>());
	return tid;

}

VlcTxNetDevice::VlcTxNetDevice(){
	m_semiangle = 0;
	m_angleOfRadiance = 0;
	m_lOrder = 1;
	m_TXGain = 0;
	m_bias = 0;
	m_TMAX = 0;
	min_rq_power = 0;

	//this->SetTxMobility();
}

VlcTxNetDevice::~VlcTxNetDevice() {

}

void ns3::VlcTxNetDevice::AttachChannel(Ptr<VlcChannel> channel) {
	m_channel = channel;
}

double VlcTxNetDevice::GetLambertianOrder() {
	return this->m_lOrder;
}

//before setting Lambertian Order make sure the semiangle value is set
//Need to setup error handling when semiangle is not set
void VlcTxNetDevice::SetLambertainOrder() {
	this->m_lOrder = (-1 * log(2)) / (log(cos(this->m_semiangle)));
}

void VlcTxNetDevice::AddTXOpticalPowerSignal(double power) {
	this->m_TXOpticalPower.push_back(power);
}

std::vector<double>& VlcTxNetDevice::GetTXOpticalPowerSignal() {
	return this->m_TXOpticalPower;
}

double VlcTxNetDevice::GetOpticalPowerSignalAtInstant(int time) {
	return this->m_TXOpticalPower.at(time);
}

void VlcTxNetDevice::SetTXOpticalPowerSignal(std::vector<double> &powerSignal) {
	this->m_TXOpticalPower = powerSignal;
}

void VlcTxNetDevice::SetTxPowerMax(double pMax) {
	this->m_TMAX = pMax;
}

double VlcTxNetDevice::GetTXPowerMax() {
	return this->m_TMAX;
}

void VlcTxNetDevice::SetCapacity(int size) {
	m_TXOpticalPower.reserve(size);
	m_signal.reserve(size);
}

double VlcTxNetDevice::GetSemiangle() {
	return this->m_semiangle;
}

void VlcTxNetDevice::SetSemiangle(double angle) {
	this->m_semiangle = angle * M_PI / 180;
	this->SetLambertainOrder();
}

void VlcTxNetDevice::SetAngleOfRadiance(double angle) {
	//std::cout << this->m_angleOfRadiance << std::endl;
	this->m_angleOfRadiance = angle * M_PI / 180;
	this->SetTXGain();
}

double VlcTxNetDevice::GetAngleOfRadiance() {
	return this->m_angleOfRadiance;
}

double VlcTxNetDevice::GetTXGain() {
	return this->m_TXGain;
}
void VlcTxNetDevice::SetTXGain() {
	//Channel loss is divided into TX gain and RX gain, which are used to calculate the loss.
	this->m_TXGain = ((this->m_lOrder + 1) / (2 * M_PI))
			* std::pow(std::cos((long double) this->m_angleOfRadiance),
					this->m_lOrder);
}

void VlcTxNetDevice::AddSignal(double signal) {
	this->m_signal.push_back(signal);
}

std::vector<double>& VlcTxNetDevice::GetSignal() {
	return this->m_signal;
}

double VlcTxNetDevice::GetSignalAtInstant(int time) {
	return this->m_signal.at(time);
}

void VlcTxNetDevice::SetSignal(std::vector<double> &signal) {
	this->m_signal = signal;
}

void VlcTxNetDevice::SetBias(double bias) {
	this->m_bias = bias;
}

double VlcTxNetDevice::GetBias() {
	return m_bias;
}

void VlcTxNetDevice::BoostSignal() {

	m_TXOpticalPower.clear();

	for (unsigned int i = 0; i < m_signal.size(); i++) {
		m_TXOpticalPower.push_back((double) (m_signal.at(i) + m_bias));
		if (m_TXOpticalPower.at(i) > m_TMAX) {
			m_TXOpticalPower.at(i) = m_TMAX;
		}
	}
}

double VlcTxNetDevice::GetAveragePowerSignalPower() {
	double pMax = *std::max_element(this->m_TXOpticalPower.begin(), this->m_TXOpticalPower.end());
	double pMin = *std::min_element(this->m_TXOpticalPower.begin(), this->m_TXOpticalPower.end());
	return (pMax + pMin) / 2;
}

double VlcTxNetDevice::GetAverageSignal() {
	double pMax = *std::max_element(this->m_signal.begin(), this->m_signal.end());
	double pMin = *std::min_element(this->m_signal.begin(), this->m_signal.end());
	return (pMax + pMin) / 2;
}

void VlcTxNetDevice::EnqueueDataPacket(Ptr<Packet> p) {
	m_channel->TransmitDataPacket(p);
}

void VlcTxNetDevice::TransmitComplete(double t) {
	m_channel->SubstractInterference(t);
}

void VlcTxNetDevice::SetPrecodingVector(std::map<uint16_t, double> precoding_term) {
	m_precoding_term = precoding_term;
}

std::map<uint16_t, double> VlcTxNetDevice::GetPrecodingVector() {
	return m_precoding_term;
}

double VlcTxNetDevice::GetPrecodingTerm(uint16_t rx_id) {
	return m_precoding_term[rx_id];
}

void VlcTxNetDevice::SetVectorPower(uint16_t rx_id, double power) {
	double total_power = 0;

	vector_power[rx_id] = power;

	//VT
	if(!m_precoding_term.empty()) {
		for(std::map<uint16_t, double>::iterator p = vector_power.begin();
				p != vector_power.end(); ++p) {
			total_power += p->second * std::pow(m_precoding_term[p->first], 2);
		}

		total_power /= (2 * M_PI);
		total_power = sqrt(total_power);
	}
	//CT
	else {
		for(std::map<uint16_t, double>::iterator p = vector_power.begin();
				p != vector_power.end(); ++p) {
			total_power += p->second;
		}
		total_power /= (2 * M_PI);
		total_power = sqrt(total_power);
	}

	if(total_power > this->GetTXPowerMax()){
		for(std::map<uint16_t, double>::iterator p = vector_power.begin();
				p != vector_power.end(); ++p) {
			std::cout << p->first << " " << p->second << std::endl;
		}
		std::cout << rx_id << " " << power << std::endl;
		std::cout << total_power << std::endl;

		for(std::map<uint16_t, double>::iterator m = m_precoding_term.begin();
						m != m_precoding_term.end(); ++m) {
			std::cout << m->first << " " << m->second << std::endl;
		}

		throw std::logic_error("Power higher than device TX Power");
	}
	if(power < 0){
		throw std::logic_error("Negative minimum power is not allowed");
	}
}

double VlcTxNetDevice::GetVectorPower(uint16_t rx_id) {
	return vector_power[rx_id];
}

double VlcTxNetDevice::GetTotalTransmitPower() {
	double total_power = 0;
	for(std::map<uint16_t, double>::iterator power = vector_power.begin();
			power != vector_power.end(); ++power) {
		total_power += power->second;
	}
	return total_power;
}

bool VlcTxNetDevice::IsPowerAllocated() {
	return !vector_power.empty();
}

void VlcTxNetDevice::SetDataRateInbps(uint16_t rx_id, double rate) {
	double total_rate = 0;
	for(std::map<uint16_t, double>::iterator r = m_data_rate.begin();
			r != m_data_rate.end(); ++r) {
		total_rate += r->second;
	}

	total_rate += rate;

	if(total_rate > this->GetTXRateMax()){
		throw std::logic_error("Rate higher than device TX Rate");
	}
	if(rate < 0){
		throw std::logic_error("Negative minimum rate is not allowed");
	}

	m_data_rate[rx_id] = rate;
}

double VlcTxNetDevice::GetDataRateInbps(uint16_t rx_id) {
	return m_data_rate[rx_id];
}

void VlcTxNetDevice::SetTXRateMax(double RMax) {
	m_RMAX = RMax;
}

double VlcTxNetDevice::GetTotalPowerConsumption() {
	double total = 0;

	if(!m_precoding_term.empty()) {
		std::map<uint16_t, double>::iterator p = vector_power.begin();
		std::map<uint16_t, double>::iterator g = m_precoding_term.begin();

		for(; p != vector_power.end() && g != m_precoding_term.end();
				++p, ++g) {
			total += std::pow(g->second, 2) * p->second;
		}
	}
	else {
		return this->GetTotalTransmitPower();
	}

	return total;
}

double VlcTxNetDevice::GetTXRateMax() {
	return m_RMAX;
}

void VlcTxNetDevice::SetMinRequiredPower(double power) {
	min_rq_power = power;
}

double VlcTxNetDevice::GetMinRequiredPower() {
	return min_rq_power;
}

void VlcTxNetDevice::Clear() {
	m_precoding_term.clear();
	vector_power.clear();
	m_data_rate.clear();
}


} /* namespace vlc */
