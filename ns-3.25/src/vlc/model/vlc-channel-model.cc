#include "ns3/vlc-channel-model.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("VlcChannel");

NS_OBJECT_ENSURE_REGISTERED (VlcChannel);

ns3::TypeId VlcChannel::GetTypeId(void)	// returns meta-information about VlcErrorModel class
		{ 	// including parent class, group name, constructor, and attributes
	static ns3::TypeId tid =
			ns3::TypeId("VlcChannel").SetParent<ns3::Channel>().AddConstructor<
					VlcChannel>().AddAttribute("AveragePower", "Average Power",
					DoubleValue(0), MakeDoubleAccessor(&VlcChannel::m_AvgPower),
					MakeDoubleChecker<double>());
	return tid;
}

VlcChannel::VlcChannel() :
	m_distanceBWTXandRX(0) {
	NS_LOG_FUNCTION(this);
	m_loss = CreateObject<VlcPropagationLossModel>();
	m_AvgPower = 0;

	m_SNR = CreateObject<VlcSnr>();
	m_nDevices = 0;
}

void ns3::VlcChannel::Attach(Ptr<VlcNetDevice> device) {
	m_link[m_nDevices++].m_src = device;
	if (m_nDevices == 2) {
		m_link[0].m_dst = m_link[1].m_src;
		m_link[1].m_dst = m_link[0].m_src;
	}
}

uint32_t VlcChannel::GetNDevices(void) const {
	return 2;
}

Ptr<NetDevice> VlcChannel::GetDevice(uint32_t i) const {
	return m_link[i].m_src;
}

void VlcChannel::SetPropagationLossModel(
		ns3::Ptr<ns3::VlcPropagationLossModel> loss) {
	NS_LOG_FUNCTION(this<<loss);
	this->m_loss = loss;
}

ns3::Ptr<ns3::VlcPropagationLossModel> VlcChannel::GetPropagationLossModel() {
	NS_LOG_FUNCTION(this);
	return this->m_loss;
}

void VlcChannel::SetPropagationDelayModel(
		ns3::Ptr<ns3::PropagationDelayModel> delay) {
	NS_LOG_FUNCTION(this<<delay);
	this->m_delay = delay;
}
ns3::Ptr<ns3::PropagationDelayModel> VlcChannel::GetPropagationDelayModel() {
	NS_LOG_FUNCTION(this);
	return this->m_delay;
}

double VlcChannel::GetDistance() {
	NS_LOG_FUNCTION(this);
	return this->m_distanceBWTXandRX;
}

void VlcChannel::SetDistance() {
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcNetDevice > first = ns3::DynamicCast < VlcNetDevice
			> (this->GetDevice(0));
	ns3::Ptr < VlcNetDevice > second = ns3::DynamicCast < VlcNetDevice
			> (this->GetDevice(1));

	ns3::Ptr < VlcPropagationLossModel > l = ns3::DynamicCast
			< VlcPropagationLossModel > (this->m_loss);
	this->m_distanceBWTXandRX = l->GetDistance(first->GetMobilityModel(),
			second->GetMobilityModel());
}

void VlcChannel::DoCalcPropagationLoss() {
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcTxNetDevice > first = ns3::DynamicCast < VlcTxNetDevice
			> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > second = ns3::DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	double loss = 0;

	for (unsigned int i = 0; i < first->GetTXOpticalPowerSignal().size(); i++) {
		loss = m_loss->CalcRxPower(first->GetTXOpticalPowerSignal().at(i),first->GetMobilityModel(), second->GetMobilityModel());
		second->GetRXOpticalPowerSignal().at(i) = loss;
	}
}

void VlcChannel::SetPropagationDelay(double delay) {
	NS_LOG_FUNCTION(this<<delay);
	//this->m_delay->set
}

void VlcChannel::SetWavelength(int lower, int upper) {// sets upper and lower bound wavelength [nm]
	NS_LOG_FUNCTION(this);
	this->m_SNR->SetWavelength(lower, upper);
}

double VlcChannel::GetDistance(ns3::Ptr<ns3::MobilityModel> aTX,
		ns3::Ptr<ns3::MobilityModel> bRX) const {
	double dist = 0;
	Vector tx = aTX->GetPosition();
	Vector rx = bRX->GetPosition();
	dist = std::pow((tx.x - rx.x), 2) + std::pow((tx.y - rx.y), 2)
			+ std::pow((tx.z - rx.z), 2);
	dist = std::sqrt(dist);
	return dist;
}


void VlcChannel::SetTemperature(double t) {	// sets the blackbody temperature of LED
	NS_LOG_FUNCTION(this<<t);
	this->m_SNR->SetTemperature(t);
}

double VlcChannel::DoCalcPropagationLossForSignal(int timeInstant) {
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcTxNetDevice > tx = ns3::DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > rx = ns3::DynamicCast < VlcRxNetDevice> (this->GetDevice(1));

	//Reset irradiance, incidence, and then concentration gain
	tx->SetAngleOfRadiance(this->GetAngleOfRadiance(tx->GetMobilityModel(), rx->GetMobilityModel()));
	rx->SetIncidenceAngle(this->GetIncidenceAngle(tx->GetMobilityModel(), rx->GetMobilityModel()));

	tx->SetTXGain();
	rx->SetRXGain();

	//set precoding vector
	std::map<uint16_t, double> tpc = tx->GetPrecodingVector();
	double precoding_term = 1.0;
	if(!tpc.empty()) {
		precoding_term = tpc[rx->GetNode()->GetId()];
	}
	//std::cout << precoding_term << std::endl;

	double TxPower = tx->GetVectorPower(rx->GetNode()->GetId());
	/*if(TxPower != 0) {
		std::cout << "Transmit power for " << rx->GetNode()->GetId() << std::endl;
		std::cout << TxPower << std::endl;
	}*/

	TxPower = sqrt(TxPower);

	double distance = this->GetDistance(tx->GetMobilityModel(), rx->GetMobilityModel());
	double RxPower = TxPower * tx->GetTXGain() * rx->GetRXGain()
			* rx->GetFilterGain() * rx->GetConcentrationGain() * rx->GetPhotoDetectorArea();
	RxPower /= std::pow(distance, 2);
	RxPower *= precoding_term;

	m_SNR->SetResponsivity(rx->GetResponsivity());

	/*if(tpc.empty()) {
		//std::cout << RxPower << std::endl;
		this->AggregateSignal(RxPower, std::to_string(tx->GetNode()->GetId()));
	}*/

	if(tx->GetMinRequiredPower() != 0) {
			//std::cout << RxPower << std::endl;
			double power = tx->GetMinRequiredPower();
			power = sqrt(power);
			this->AggregateSignal(power, std::to_string(tx->GetNode()->GetId()));
		}
	else
		this->AggregateSignal(TxPower, std::to_string(tx->GetNode()->GetId()));

//******************************************************************************//

	//other channels' snr (in the same cluster) should be updated as well
	for(std::map<std::string, ns3::Ptr<VlcChannel> >::iterator it = m_other_channel.begin()
  		; it != m_other_channel.end(); ++it) {
		ns3::Ptr < VlcRxNetDevice > rx_other = DynamicCast < VlcRxNetDevice> (it->second->GetDevice(1));

		if(rx_other->GetNode()->GetId() != rx->GetNode()->GetId()) {
			//for zero-forcing VT
			if(!tpc.empty()) {
				for(std::map<uint16_t, double>::iterator p = tpc.begin(); p != tpc.end();
						++p) {
					//if UE is in the same cluster
					if(p->first == rx_other->GetNode()->GetId()) {

						//null the interference using precoding term
						tx->SetAngleOfRadiance(this->GetAngleOfRadiance(tx->GetMobilityModel(),
								rx_other->GetMobilityModel()));
						rx_other->SetIncidenceAngle(this->GetIncidenceAngle(tx->GetMobilityModel(),
								rx_other->GetMobilityModel()));
						tx->SetTXGain();
						rx_other->SetRXGain();

						//set precoding vector

						double distance = this->GetDistance(tx->GetMobilityModel(), rx_other->GetMobilityModel());
						double interf_power = TxPower * tx->GetTXGain() * rx_other->GetRXGain()
								* rx_other->GetFilterGain() * rx_other->GetConcentrationGain() * rx_other->GetPhotoDetectorArea();
						interf_power /= std::pow(distance, 2);
						RxPower *= precoding_term;

						interf_power *= precoding_term;

						std::ostringstream chName;
						chName << tx->GetNode()->GetId() << "-" << rx->GetNode()->GetId();
						//it->second->AddInterference(interf_power, chName.str());
					}

				}
			}
		}

	}

	return RxPower;
}


double VlcChannel::GetAngleOfRadiance (Ptr<VlcMobilityModel> tx, Ptr<VlcMobilityModel> rx) const
{
	//TODO: expansion for different device orientation
	double angle = 0;

	Vector AP_pos = tx->GetPosition();
	Vector UE_pos = rx->GetPosition();

	double height_diff = AP_pos.z - UE_pos.z;
	double plane_dist = sqrt(pow(AP_pos.x - UE_pos.x, 2) + pow(AP_pos.y - UE_pos.y, 2));
	double hypotenuse = CalculateDistance(AP_pos, UE_pos);

	 // angle of incidence = angle between <height diff> and <hypotenuse>
	angle = acos((pow(height_diff,2) + pow(hypotenuse,2) - pow(plane_dist,2)) / (2*height_diff*hypotenuse)) * 180 / M_PI;

	return angle;
}

double VlcChannel::GetIncidenceAngle (Ptr<VlcMobilityModel> tx, Ptr<VlcMobilityModel> rx) const
{
	//TODO: expansion for different device orientation
	double angle = 0;

	Vector AP_pos = tx->GetPosition();
	Vector UE_pos = rx->GetPosition();

	double height_diff = AP_pos.z - UE_pos.z;
	double plane_dist = sqrt(pow(AP_pos.x - UE_pos.x, 2) + pow(AP_pos.y - UE_pos.y, 2));
	double hypotenuse = CalculateDistance(AP_pos, UE_pos);

	 // angle of incidence = angle between <height diff> and <hypotenuse>
	angle = acos((pow(height_diff,2) + pow(hypotenuse,2) - pow(plane_dist,2)) / (2*height_diff*hypotenuse)) * 180 / M_PI;

	return angle;
}
double VlcChannel::GetTemperature() {
	NS_LOG_FUNCTION(this);
	return this->m_SNR->GetTemperature();
}

void VlcChannel::CalculateNoiseVar() {	//calculates the noise variance
	NS_LOG_FUNCTION(this );
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	this->m_SNR->CalculateNoiseVar(rx->GetPhotoDetectorArea());

}
void VlcChannel::CalculateSNR() {		// caluclates the SNR value
	NS_LOG_FUNCTION(this);
	this->m_SNR->CalculateSNR();
}

void VlcChannel::SetReceivedPower(double p) {// sets the average received optical signal power
	NS_LOG_FUNCTION(this<<p);
	this->m_SNR->SetReceivedPower(p);
}

double VlcChannel::GetSNR() const {	// returns the signal-to-noise ratio (SNR)
	NS_LOG_FUNCTION(this);
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice
			> (this->GetDevice(1));
	this->m_SNR->CalculateNoiseVar(rx->GetPhotoDetectorArea());
	m_SNR->CalculateSNR();
	return this->m_SNR->GetSNR();
}

void VlcChannel::SetAveragePower(double power) {
	NS_LOG_FUNCTION(this<<power);
	m_AvgPower = power;
}

double VlcChannel::GetAveragePower() {
	NS_LOG_FUNCTION(this);
	return m_AvgPower;
}

void VlcChannel::SetElectricNoiseBandWidth(double b) {// sets the noise bandwidth
	NS_LOG_FUNCTION(this<<b);
	this->m_SNR->SetElectricNoiseBandWidth(b);
}

double VlcChannel::GetNoiseBandwidth() {			//return the noise bandwidth
	NS_LOG_FUNCTION(this);
	return m_SNR->GetNoiseBandwidth();
}

void VlcChannel::AddInterference(double power, std::string chName) {
	//which channel caused the interference
	m_SNR->AddInterference(chName, power);
}

void VlcChannel::AggregateSignal(double power, std::string txName) {
	//which AP aggregated this signal
	m_SNR->AggregateSignal(txName, power);
}

void VlcChannel::SubstractInterference(double t) {
	/*ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	std::string devName = std::to_string(tx->GetNode()->GetId());

	for(std::map<std::string, ns3::Ptr<VlcChannel> >::iterator it =m_other_channel.begin()
	  		; it != m_other_channel.end(); ++it) {
		ns3::Ptr<VlcChannel> channel = it->second;
		ns3::Ptr < VlcRxNetDevice > rx_interf = DynamicCast < VlcRxNetDevice> (it->second->GetDevice(1));

		//not work - not the correct timing
		Simulator::ScheduleWithContext(rx_interf->GetNode()->GetId(),
				Seconds(t), &VlcChannel::AddInterference, channel, 0);
	}*/
}

void VlcChannel::TransmitDataPacket(Ptr<Packet> p) {
	NS_LOG_FUNCTION(this<<p);
	//this is the point where we decide to keep the packet or corrupt it
	this->DoCalcPropagationLossForSignal(0);
	this->CalculateNoiseVar();

	ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> (this->GetDevice(1));
	if(rx->GetTime() != Simulator::Now().GetSeconds()) {
		rx->GetErrorModel()->SetSNR(this->GetSNR()); //for each transmission event calculate the SNR and set it in error model.
		double distance = this->GetDistance(tx->GetMobilityModel(),	rx->GetMobilityModel());
		//std::cout << this->GetSNR() << std::endl;
		ns3::Ptr < ns3::VlcErrorModel > rxErrorModel = rx->GetErrorModel();

		double erRate = rxErrorModel->CalculateErrorRate();
		//std::cout << rx->GetNode()->GetId() << std::endl;
		/*if(erRate > 1e-5) {
			std::cout << "error rate: " << erRate << std::endl;
		}*/

		//int size = p->GetSize();
		//double packetErrorRate = 1.0 - std::pow((1 - erRate), 2048);

		//std::ofstream pers;
		//pers.open("packetErrorRateFile.txt", std::ios_base::app);
		//pers << distance << "\t" << packetErrorRate << std::endl;
		//std::cout << distance << "\t" << std::endl;
		bool isCorrupt = rxErrorModel->CorruptPacket(p, erRate);
		//std::cout << packetErrorRate<< std::endl;
		rx->EnqueueDataPacketAfterCorruption(p, isCorrupt);
		//pers.close();

		rx->SetTime(Simulator::Now().GetSeconds());
	}

	TransmitStart(p, tx, Simulator::Now());

}

bool VlcChannel::TransmitStart(Ptr<Packet> p, Ptr<VlcNetDevice> src,
		Time txTime) {
	NS_LOG_FUNCTION(this<<p<<src);
	//static int i = 0;
	this->DoCalcPropagationLossForSignal(0);
	this->CalculateNoiseVar();

	ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> (this->GetDevice(0));
	ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> (this->GetDevice(1));

	for(std::map<std::string, ns3::Ptr<VlcChannel> >::iterator it = m_other_channel.begin()
  		; it != m_other_channel.end(); ++it) {

		ns3::Ptr < VlcRxNetDevice > rx_other = DynamicCast < VlcRxNetDevice> (it->second->GetDevice(1));
		//if not in the same cluster
		std::map<uint16_t, double> tpc = tx->GetPrecodingVector();
		bool IsSameCluster = false;

		for(std::map<uint16_t, double>::iterator p = tpc.begin(); p != tpc.end();
				++p) {
			if(p->first == rx_other->GetNode()->GetId()){
				IsSameCluster = true;
				break;
			}
		}
		//add interference
		if(!IsSameCluster && rx_other->GetNode()->GetId() != rx->GetNode()->GetId()) {
			//received power calculation
			tx->SetAngleOfRadiance(this->GetAngleOfRadiance(tx->GetMobilityModel(), rx_other->GetMobilityModel()));
			rx_other->SetIncidenceAngle(this->GetIncidenceAngle(tx->GetMobilityModel(), rx_other->GetMobilityModel()));

			tx->SetTXGain();
			rx_other->SetRXGain();

			double TxPower = tx->GetVectorPower(rx->GetNode()->GetId());
			TxPower = sqrt(TxPower);
			double distance = this->GetDistance(tx->GetMobilityModel(), rx_other->GetMobilityModel());
			double interf_power = TxPower * tx->GetTXGain() * rx_other->GetRXGain()
					* rx_other->GetFilterGain() * rx_other->GetConcentrationGain() * rx_other->GetPhotoDetectorArea();
			interf_power /= std::pow(distance, 2);

			ns3::Ptr<VlcChannel> channel = it->second;

			std::ostringstream chName;
			chName << tx->GetNode()->GetId() << "-" << rx->GetNode()->GetId();

			channel->AddInterference(interf_power, chName.str());
		}

	}

	m_delay = CreateObject<ConstantSpeedPropagationDelayModel>();

	rx->GetErrorModel()->SetSNR(this->GetSNR()); //for each transmission event calculate the SNR and set it in error model.

	uint32_t id = src == m_link[0].m_src ? 0 : 1;

	Time m_delayTime = m_delay->GetDelay(tx->GetMobilityModel(),
			rx->GetMobilityModel());

	//std::cout << Simulator::Now().GetSeconds() << std::endl;

	/*Simulator::ScheduleWithContext(m_link[id].m_dst->GetNode()->GetId(),
	txTime + m_delayTime, &VlcNetDevice::Receive, m_link[id].m_dst, p);*/

	// Call the tx anim callback on the net device
	m_txrxVlcChannel(p, src, m_link[id].m_dst, txTime, txTime + m_delayTime);

	return true;
}

void VlcChannel::InstallOtherChannel(std::pair<std::string, ns3::Ptr<VlcChannel>> other) {
	m_other_channel.insert(other);
}

void VlcChannel::AttachPropogationLossModel(Ptr < VlcNetDevice > tx, Ptr < VlcNetDevice > rx) {
	ns3::Ptr < VlcTxNetDevice > TX = ns3::DynamicCast < VlcTxNetDevice> (tx);
	ns3::Ptr < VlcRxNetDevice > RX = ns3::DynamicCast < VlcRxNetDevice> (rx);

	m_loss->SetTxPowerMAX(TX->GetTXPowerMax());
	m_loss->SetTXGain(TX->GetTXGain());

	m_loss->SetFilterGain(RX->GetFilterGain());

	m_loss->SetConcentratorGain(
			RX->GetConcentrationGain());

	m_loss->SetRXGain(RX->GetRXGain());

	m_loss->SetArea(RX->GetPhotoDetectorArea());
}

void VlcChannel::AccumulatePacketCount() {
	packet_count++;
}

int VlcChannel::GetPacketCount() {
	return packet_count;
}

/*double VlcChannel::DoCalcRxPower(double txPowerDbm,
		ns3::Ptr<VlcTxNetDevice> aTX,
		ns3::Ptr<VlcNetDevice> bRX) const {

	aTX->SetAngleOfRadiance(this->GetAngleOfRadiance(aTX->GetMobilityModel(), bRX->GetMobilityModel()));
	bRX->SetIncidenceAngle(this->GetIncidenceAngle(aTX->GetMobilityModel(), bRX->GetMobilityModel()));

	aTX->SetTXGain();
	bRX->SetRXGain();

	double TxGain = aTX->GetTXGain();
	double RxGain = bRX->GetRXGain();

	double distance = this->GetDistance(aTX->GetMobilityModel(), bRX->GetMobilityModel());
	double pRX = txPowerDbm * RxGain * TxGain
			* bRX->GetFilterGain() * bRX->GetConcentrationGain() * bRX->GetPhotoDetectorArea();
	pRX /= std::pow(distance, 2);

	return pRX;
}*/

VlcChannel::~VlcChannel() {

}

} /* namespace vlc */
