#ifndef VlcSnr_H
#define VlcSnr_H

#include "ns3/object.h"
#include <map>

namespace ns3 {

class VlcSnr: public Object	// This class is a subclass of Object class
{
public:
	// Public methods
	static TypeId GetTypeId(void); // returns meta-information about PAMErrorModel class 

	VlcSnr();	// constructor
	virtual ~VlcSnr();	//destructor

	void SetWavelength(int lower, int upper);// sets upper and lower bound wavelength [nm]
	void SetTemperature(double t);		// sets the blackbody temperature of LED
	double GetTemperature();
	void SetReceivedPower(double p);// sets the average received optical signal power
	void SetElectricNoiseBandWidth(double b);	// sets the noise bandwidth
	double GetNoiseBandwidth();			//return the noise bandwidth

	void CalculateNoiseVar(double A);	//calculates the noise variance
	void CalculateSNR();		// caluclates the SNR value
	double GetSNR();		// returns the signal-to-noise ratio (SNR)

	void AddInterference(std::string txName, double power);
	void AggregateSignal(std::string txName, double power);

	void SetResponsivity(double r);


private:
	// Private methods called by other methods inside the class
	double IntegralPlanck();
	double IntegralRes();
	double SpectralRadiance(int wavelength, double temperature);

	// Private class members
	double noise_var;   // total noise variance
	double Pr;			// average received optical signal power
	double res;  		// responsitivity of receiver
	double SNR;  		// signal-to-noise ratio
	double B;			// B: electric noise bandwidth

	int wavelength_lower;	// lower bound WaveLength [nm]
	int wavelength_upper;	// upper bound Wavelength [nm]
	double temp;  			// Blackbody temp of LED

	std::map<std::string, double> interference;
	std::map<std::string, double> signal;

	static double V_lambda[];
	static double Response[];

	double noise_spectral_density = 1e-22;
	double bw = 20 * 1e6; //20MHz
};

} // namespace ns3
#endif
