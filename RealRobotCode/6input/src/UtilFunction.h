
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <cmath>
#include <sstream>
using namespace std;

class CEstimateCollisionNN
{
public:
	
	ifstream file[7];
	ofstream writeFile;

    CEstimateCollisionNN();
    ~CEstimateCollisionNN();

	void EstimateCollision(float dInput[], double dOutput[]);
	void SetNNWeight(); ///< ����� �ۼ�
	void SetNNBias(); ///< ����� �ۼ�
	void SetBNGamma(); ///< ����� �ۼ�
	void SetBNBeta(); ///< ����� �ۼ�
	void SetBNMean(); ///< ����� �ۼ�
	void SetBNVariance(); ///< ����� �ۼ�
	void SetInputNormalMinMax(); ///< ����� �ۼ�

    /********* Notch filter **********/
private:

	float m_dWeight[6000];
	float m_dBias[350];
	float m_dGamma[350];
	float m_dBeta[350];
	float m_dMean[350];
	float m_dVariance[350];

	void InputNormalization(float dInput[], float dInputN[]);
	float m_dInputN[155];
	float m_dInputNormalMin[31];
	float m_dInputNormalMax[31];

	void CalLayer1(float dInput[], float dOutput[]);
	float m_dLayer1[105];

	void CalLayer2(float dInput[], float dOutput[]);
	float m_dLayer2[105];

	void CalLayer3(float dInput[], float dOutput[]);
	float m_dLayer3[91];

	void CalLayer4(float dInput[], float dOutput[]);
	float m_dLayer4[7];

	void CalLayer5(float dInput[], float dOutput[]);
	float m_dLayer5[15];

	void CalLayer6(float dInput[], float dOutput[]);
	float m_dLayer6[15];

	void CalOutput(float dInput[], double dOutput[]);
};
