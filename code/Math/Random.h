#pragma once
#include <random>

class Random
{
public :
	static float Get() { return m_distribution(m_generator); }

private:
	static std::mt19937 m_generator;
	static std::uniform_real_distribution<float> m_distribution;
};



// Test with Pi
float Pi_Uniform(int sampleX, int sampleY);
float Pi_MonteCarlo(int numSamples);
void TestPi();