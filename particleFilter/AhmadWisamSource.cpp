#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <cmath>
#include <Windows.h>

using namespace std;
/*				 	    - Palestine Polytechnic University -
						College of IT and Computer Egineering
						 Intelligent Systems Course Project
							Particle Filter Simulation
						Group:
							1. Ahmad Horayzat
							2. Wisam Alhroub
*/


//necessary variables:
const int particleNum = 120;
double y[1001], totalW, avg, stD;
const double PI = 3.14159;
const double Exp = 2.7183;

//necessary functions:
void yEquation();
void buildBiasedRandom(double br[particleNum + 1], double s[][particleNum]);
int generateBiasedRandom(double br[particleNum + 1], int ran);
void updatePosition(double s[][particleNum], double sNew[][particleNum], int i, int index, double u);
void updateWeight(double sNew[][particleNum], int i, double diff);
void particleFilter(double s[][particleNum], int u, double z);
void print(double s[][particleNum], int i, int currentPosition);

int main() {

	double s[2][particleNum];
	yEquation(); //generates the values of the sensor reading across the wall

	//Calculating the average:
	double sum = 0.0;
	for (int i = 0; i < 1001; i++)
		sum += y[i];
	avg = sum / 1001;

	//Calculating the standard deviation:
	stD = 0.0;
	for (int i = 0; i < 1001; i++)
		stD += pow(y[i] - avg, 2);
	stD = sqrt(stD / 1001);


	int currentPosition = 130;
	const int stepSize = 10;

	//Initialising the particles:
	int j = 0;
	int x = 1000 / particleNum;
	for (int i = 0; i < particleNum; i++) {
		if (j + x <= 1000) {
			s[0][i] = j + (x * 1.0);
			s[1][i] = 1.0 / particleNum;
			j += x;

		}
		else {
			s[0][i] = rand() % 1000;
			s[1][i] = 1.0 / particleNum;
			j += x;
		}
	}

	//Starting the particle filter algorithm:
	double robotTemp;
	int i = 0;
	while (currentPosition < 1000) {

		robotTemp = y[currentPosition]
			+ (rand() % 11) / 1000.0 - 0.005; //error rate of [-0.005, 0.005]
		particleFilter(s, stepSize, robotTemp);

		currentPosition += stepSize
			+ rand() % (int)(0.5 * stepSize) - 2; //error rate proportional to the step size range 20% of step size
		if (currentPosition > 1000)
			currentPosition = 1000;
		print(s, i, currentPosition);

		i++;
		Sleep(1000); // waiting time before next output in ms

	}

	return 0;
}


void yEquation() {
	double t = 0.0;
	double x = 0.0;

	for (int i = 0; i <= 1000; i++) {
		x = t;
		x *= 2 * PI;

		y[i] = cos(x)
			+ 0.5 * cos(3 * x + 0.23)
			+ 0.5 * cos(5 * x - 0.4)
			+ 0.5 * cos(7 * x + 2.09)
			+ 0.5 * cos(9 * x - 3);
		t += 0.001;
	}
}

void buildBiasedRandom(double br[particleNum + 1], double s[][particleNum]) {
	br[0] = 0; int cumulative = 0;
	double w;
	for (int i = 1; i < particleNum + 1; i++) {
		w = s[1][i - 1];
		w *= 100000;
		w = round(w); //returns the approximation of a float number
		w /= 100000.0; // set precision of weight to 0.00000;
		br[i] = s[1][i - 1] * 100000 + cumulative;
		cumulative += w * 100000;

	}
}

int generateBiasedRandom(double br[particleNum + 1], int ran) {
	int i;
	for (i = 1; i < particleNum; i++) {
		if (ran < br[i])
			break;
	}
	return i - 1;
}

void updatePosition(double s[][particleNum], double sNew[][particleNum], int i, int index, double u) {
	int er = rand() % (int)(0.5 * u) - 2;
	if ((s[0][index] + u + er) <= 1000)
		sNew[0][i] = (int)s[0][index] + u + er;
	else
		sNew[0][i] = 1000;
}

void updateWeight(double sNew[][particleNum], int i, double diff) {
	sNew[1][i] = pow(Exp, -1 * pow(diff - avg, 2) / (2 * stD * stD)) / stD * sqrt(2 * PI);
	totalW += sNew[1][i];
}

void particleFilter(double s[][particleNum], int u, double z) {
	double diff;
	double br[particleNum + 1];
	totalW = 0.0;
	buildBiasedRandom(br, s); //builds the array of the biased randoms(br)
	double sNew[2][particleNum];

	//initialization of sNew
	for (int i = 0; i < particleNum; i++) {
		sNew[0][i] = rand() % 1000;
		sNew[1][i] = 1.0 / particleNum;
	}


	for (int i = 0; i < particleNum; i++) {
		int rnd = rand() % 100000;

		int gr = generateBiasedRandom(br, rnd); //Select a random with bias to the weights

		if (s[0][gr] + u <= 1000) {
			diff = z - y[(int)s[0][gr]] + (rand() % 11) / 1000.0 - 0.005;
			diff = abs(diff);
			updatePosition(s, sNew, i, gr, u);
			updateWeight(sNew, i, diff);
		}
		else {
			int gr2 = generateBiasedRandom(br, rand() % 100) * 1000.0 / particleNum;
			diff = z - y[(int)s[0][gr]] + (rand() % 11) / 1000.0 - 0.005;
			diff = abs(diff);
			updatePosition(s, sNew, i, gr2, u);
			updateWeight(sNew, i, diff);
		}
	}

	for (int i = 0; i < particleNum; i++) {
		sNew[1][i] /= totalW;
		s[0][i] = sNew[0][i];
		s[1][i] = sNew[1][i];
	}

}

void print(double s[][particleNum], int i, int currentPosition) {
	double mean = 0; int k = 0;
	cout << " __________________________________\n";
	for (int i = 0; i < particleNum; i++) {
		cout << '|' << setw(6) << s[0][i]; k++;
		mean += s[0][i];
		if (k % 5 == 0)
			cout << "|\n|______|______|______|______|______|\n";
	}
	mean /= particleNum;
	double stDP = 0;
	for (int i = 0; i < particleNum; i++) {
		stDP += pow((s[0][i] - mean), 2);
	}
	stDP /= particleNum;
	stDP = sqrt(stDP);
	cout << "\nParticles position average: " << mean << '\n';
	cout << "Particles Standard deviation: " << stDP << '\n';

	cout << "Robot position: " << currentPosition
		<< "\n*********************************\n";
	cout << "-----------------\nIteration #" << i + 1 << ":\n-----------------\n";
}
