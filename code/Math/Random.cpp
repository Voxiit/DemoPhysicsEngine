#include "Random.h"

std::mt19937 Random::m_generator(0);
std::uniform_real_distribution<float> Random::m_distribution(0.0f, 1.0f);


// Test with Pi
float Pi_Uniform(int sampleX, int sampleY)
{
	int counter = 0;
	for (int y = 0; y < sampleY; y++)
	{
		for (int x = 0; x < sampleX; x++)
		{
			float xx = float(x) / float(sampleX);
			float yy = float(y) / float(sampleY);
			float x2 = xx * xx;
			float y2 = yy * yy;
			if (x2 + y2 < 1.0f)
			{
				counter++;
			}
		}
	}
	float pi = 4.0f * float(counter) / float(sampleX * sampleY);
	return pi;
}


float Pi_MonteCarlo(int numSamples)
{
	int counter = 0;

	for (int i = 0; i < numSamples; i++)
	{
		float xx = Random::Get();
		float yy = Random::Get();
		float x2 = xx * xx;
		float y2 = yy * yy;
		if (x2 + y2 < 1.0f)
		{
			counter++;
		}
	}
	float pi = 4.0f * float(counter) / float(numSamples);
	return pi;
}


void TestPi()
{
	int samples[4] = { 10, 100, 1000, 10000 };
	for (int i = 0; i < 4; i++)
	{
		float pi_uniform = Pi_Uniform(samples[i], samples[i]);
		float pi_monte = Pi_MonteCarlo(samples[i] * samples[i]);
		int totalSamples = samples[i] * samples[i];
		printf("uniform: %f     monte carlo: %f     Total Samples: %i\n", pi_uniform, pi_monte, totalSamples);
	}
}