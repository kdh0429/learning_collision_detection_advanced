
#include "UtilFunction.h"

CEstimateCollisionNN::CEstimateCollisionNN()
{
	for (unsigned int i=0;i<6000;i++){
		m_dWeight[i] = 0.0;
	}
	for (unsigned int i=0;i<350;i++){
		m_dBias[i] = 0.0;
	}
	for (unsigned int i=0;i<350;i++){
		m_dGamma[i] = 0.0;
	}
	for (unsigned int i=0;i<350;i++){
		m_dBeta[i] = 0.0;
	}
	for (unsigned int i=0;i<350;i++){
		m_dMean[i] = 0.0;
	}
	for (unsigned int i=0;i<350;i++){
		m_dVariance[i] = 0.0;
	}
	for (unsigned int i=0;i<31;i++){
		m_dInputNormalMin[i] = 0.0;
		m_dInputNormalMax[i] = 0.0;
	}
	
	file[0].open("Weight.txt", ios::in);
	file[1].open("Bias.txt", ios::in);
	file[2].open("Gamma.txt", ios::in);
	file[3].open("Beta.txt", ios::in);
	file[4].open("Mean.txt", ios::in);
	file[5].open("Variance.txt", ios::in);
	file[6].open("InputMinMax.txt", ios::in);

	SetNNWeight(); ///< ����� �ۼ�
	SetNNBias(); ///< ����� �ۼ�
	SetBNGamma(); ///< ����� �ۼ�
	SetBNBeta(); ///< ����� �ۼ�
	SetBNMean(); ///< ����� �ۼ�
	SetBNVariance(); ///< ����� �ۼ�
	SetInputNormalMinMax(); ///< ����� �ۼ�
	writeFile.open("Result.txt");
}

CEstimateCollisionNN::~CEstimateCollisionNN()
{
}

void CEstimateCollisionNN::EstimateCollision(float dInput[], float dOutput[])
{
	///< Input nomalization
	InputNormalization(dInput, m_dInputN);
	
	///< Layer 1
	CalLayer1(m_dInputN,m_dLayer1);

	///< Layer 2
	CalLayer2(m_dLayer1,m_dLayer2);

	///< Layer 3
	CalLayer3(m_dLayer2,m_dLayer3);

	///< Layer 4
	CalLayer4(m_dLayer3,m_dLayer4);

	///< Layer 5
	CalLayer5(m_dLayer4,m_dLayer5);

	///< Layer 6
	CalLayer6(m_dLayer5,m_dLayer6);

	///< Output
	CalOutput(m_dLayer6,dOutput);

	writeFile<< dOutput[0]<<'\n';
}

void CEstimateCollisionNN::InputNormalization(float dInput[], float dInputN[])
{
	for (unsigned int i=0;i<150;i++){
		int joint_num = int(i/25);
		int joint_data_idx = i-25*joint_num;
		int data_type = joint_data_idx % 5;
		dInputN[i] = 2*(dInput[i] - m_dInputNormalMin[5*joint_num + data_type])/(m_dInputNormalMax[5*joint_num + data_type] - m_dInputNormalMin[5*joint_num + data_type]) -1;
	}
	for (unsigned int i=150;i<155;i++){
		dInputN[i] = (dInput[i] - m_dInputNormalMin[30])/(m_dInputNormalMax[30] - m_dInputNormalMin[30]);
	}
}

void CEstimateCollisionNN::CalLayer1(float dInput[], float dOutput[])
{
	float dOutputNN[105] = {0.0};
	///< Calculation Neural Network
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[nJointDOF * 15 + nOutputNode] = m_dBias[nJointDOF * 46 + nOutputNode];
			for (unsigned int nInputNode = 0; nInputNode < 25; nInputNode++){
				dOutputNN[nJointDOF * 15 + nOutputNode] += m_dWeight[nJointDOF * 840 + nOutputNode * 25 + nInputNode] * dInput[nJointDOF * 25 + nInputNode];
			}
		}
	}
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[90+nOutputNode] = m_dBias[6 * 46 + nOutputNode];
		for (unsigned int nInputNode = 0; nInputNode < 5; nInputNode++){
			dOutputNN[90 + nOutputNode] += m_dWeight[6 * 840 + nOutputNode * 5 + nInputNode] * dInput[6 * 25 + nInputNode];
		}
	}
	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[15*nJointDOF+nOutputNode] = m_dGamma[46*nJointDOF + nOutputNode] * (dOutputNN[nJointDOF * 15 + nOutputNode] - m_dMean[46*nJointDOF + nOutputNode])/sqrt(m_dVariance[46*nJointDOF + nOutputNode]+0.001) + m_dBeta[46*nJointDOF + nOutputNode];
		}
	}
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[15*6+nOutputNode] = m_dGamma[46*6 + nOutputNode] * (dOutputNN[6 * 15 + nOutputNode] - m_dMean[46*6 + nOutputNode])/sqrt(m_dVariance[46*6 + nOutputNode]+0.001) + m_dBeta[46*6 + nOutputNode];
	}

	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 105; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}

}


void CEstimateCollisionNN::CalLayer2(float dInput[], float dOutput[])
{
	float dOutputNN[105] = {0.0};
	///< Calculation Neural Network
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[nJointDOF * 15 + nOutputNode] = m_dBias[15 + nJointDOF * 46 + nOutputNode];
			for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
				dOutputNN[nJointDOF * 15 + nOutputNode] += m_dWeight[375 + nJointDOF * 840 + nOutputNode * 15 + nInputNode] * dInput[nJointDOF * 15 + nInputNode];
			}
		}
	}
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[90+nOutputNode] = m_dBias[6 * 46 + 15 + nOutputNode];
		for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
			dOutputNN[90 + nOutputNode] += m_dWeight[6 * 840 + 5*15 + nOutputNode * 15 + nInputNode] * dInput[6 * 15 + nInputNode];
		}
	}
	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[15*nJointDOF+nOutputNode] = m_dGamma[15 + 46*nJointDOF + nOutputNode] * (dOutputNN[nJointDOF * 15 + nOutputNode] - m_dMean[15+ 46*nJointDOF + nOutputNode])/sqrt(m_dVariance[15+46*nJointDOF + nOutputNode]+0.001) + m_dBeta[15+46*nJointDOF + nOutputNode];
		}
	}
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[15*6+nOutputNode] = m_dGamma[46*6 + 15 + nOutputNode] * (dOutputNN[6 * 15 + nOutputNode] - m_dMean[46*6 + 15 + nOutputNode])/sqrt(m_dVariance[46*6 + 15 + nOutputNode]+0.001) + m_dBeta[46*6 + 15 + nOutputNode];
	}
	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 105; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}


}


void CEstimateCollisionNN::CalLayer3(float dInput[], float dOutput[])
{
	float dOutputNN[91] = {0.0};

	///< Calculation Neural Network
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[nJointDOF * 15 + nOutputNode] = m_dBias[30 + nJointDOF * 46 + nOutputNode];

			for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
				dOutputNN[nJointDOF * 15 + nOutputNode] += m_dWeight[600 + nJointDOF * 840 + nOutputNode * 15 + nInputNode] * dInput[nJointDOF * 15 + nInputNode];
			}
		}
	}
	dOutputNN[90] = m_dBias[6 * 46 + 30];
	for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
		dOutputNN[90] += m_dWeight[6 * 840 + 5*15 + 15*15 + nInputNode] * dInput[6 * 15 + nInputNode];
	}

	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
			dOutputNN[15*nJointDOF+nOutputNode] = m_dGamma[30 + 46*nJointDOF + nOutputNode] * (dOutputNN[nJointDOF * 15 + nOutputNode] - m_dMean[30+ 46*nJointDOF + nOutputNode])/sqrt(m_dVariance[30+46*nJointDOF + nOutputNode]+0.001) + m_dBeta[30+46*nJointDOF + nOutputNode];
		}
	}
	dOutputNN[90] = m_dGamma[46*6 + 30] * (dOutputNN[6 * 15] - m_dMean[46*6 + 30])/sqrt(m_dVariance[46*6 + 30]+0.001) + m_dBeta[46*6 + 30];
	
	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 91; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}
}


void CEstimateCollisionNN::CalLayer4(float dInput[], float dOutput[])
{
	float dOutputNN[7] = {0.0};
	///< Calculation Neural Network
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		dOutputNN[nJointDOF] = m_dBias[45 + nJointDOF * 46];
		for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
			dOutputNN[nJointDOF] += m_dWeight[825 + nJointDOF * 840 + nInputNode] * dInput[nJointDOF * 15 + nInputNode];
		}
	}
	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nJointDOF = 0;nJointDOF < 6; nJointDOF++){
		dOutputNN[nJointDOF] = m_dGamma[45 + 46*nJointDOF] * (dOutputNN[nJointDOF] - m_dMean[45+ 46*nJointDOF])/sqrt(m_dVariance[45+46*nJointDOF]+0.001) + m_dBeta[45+46*nJointDOF];
	}
	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 6; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}
	dOutput[6] = dInput[90];
}

void CEstimateCollisionNN::CalLayer5(float dInput[], float dOutput[])
{
	float dOutputNN[15] = {0.0};

	///< Calculation Neural Network
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[nOutputNode] = m_dBias[307 + nOutputNode];

		for (unsigned int nInputNode = 0; nInputNode < 7; nInputNode++){
			dOutputNN[nOutputNode] += m_dWeight[5355 + nOutputNode * 7 + nInputNode] * dInput[nInputNode];
		}
	}

	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[nOutputNode] = m_dGamma[307 + nOutputNode] * (dOutputNN[nOutputNode] - m_dMean[307 + nOutputNode])/sqrt(m_dVariance[307 + nOutputNode]+0.001) + m_dBeta[307 + nOutputNode];
	}
	
	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 15; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}
}

void CEstimateCollisionNN::CalLayer6(float dInput[], float dOutput[])
{
	float dOutputNN[15] = {0.0};

	///< Calculation Neural Network
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[nOutputNode] = m_dBias[322 + nOutputNode];

		for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
			dOutputNN[nOutputNode] += m_dWeight[5460 + nOutputNode * 15 + nInputNode] * dInput[nInputNode];
		}
	}

	///< Calculation Batch Normalization (����� �ۼ�)
	for (unsigned int nOutputNode = 0; nOutputNode < 15; nOutputNode++){
		dOutputNN[nOutputNode] = m_dGamma[322 + nOutputNode] * (dOutputNN[nOutputNode] - m_dMean[322 + nOutputNode])/sqrt(m_dVariance[322 + nOutputNode]+0.001) + m_dBeta[322 + nOutputNode];
	}
	
	///< Calculation Activation Function (RELU)
	for (unsigned int i = 0;i < 15; i++){
		if (dOutputNN[i] < 0.0){
			dOutput[i] = 0.0;
		}
		else{
			dOutput[i] = dOutputNN[i];
		}
	}
}


void CEstimateCollisionNN::CalOutput(float dInput[], float dOutput[])
{
	float dOutputNN[2] = {0.0};
	///< Calculation Neural Network
	for (unsigned int nOutputNode = 0; nOutputNode < 2; nOutputNode++){
		dOutputNN[nOutputNode] = m_dBias[337 + nOutputNode];
		for (unsigned int nInputNode = 0; nInputNode < 15; nInputNode++){
			dOutputNN[nOutputNode] += m_dWeight[5685 + nOutputNode * 15 + nInputNode] * dInput[nInputNode];
		}
	}
	float Max = 0.0;//max(dOutputNN[0],dOutputNN[1]);
	dOutput[0] = exp(dOutputNN[0]-Max)/(exp(dOutputNN[0]-Max) + exp(dOutputNN[1]-Max));
	dOutput[1] = exp(dOutputNN[1]-Max)/(exp(dOutputNN[0]-Max) + exp(dOutputNN[1]-Max));
}


void CEstimateCollisionNN::SetNNWeight()
{
	if(!file[0].is_open())
	{
		std::cout<<"can not found the Weight file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[0].eof())
	{
		file[0] >> temp;
		if(temp != '\n')
		{
			m_dWeight[index%6000] = temp;
			index ++;
		}
	}
}

void CEstimateCollisionNN::SetNNBias()
{
	if(!file[1].is_open())
	{
		std::cout<<"can not found the Bias file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[1].eof())
	{
		file[1] >> temp;
		if(temp != '\n'){
			m_dBias[index%350] = temp;
			index ++; 
		}
	}
}

void CEstimateCollisionNN::SetBNGamma()
{
	if(!file[2].is_open())
	{
		std::cout<<"can not found the Gamma file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[2].eof())
	{
		file[2] >> temp;
		if(temp != '\n'){
			m_dGamma[index%350] = temp;
			index ++; 
		}
	}
}

void CEstimateCollisionNN::SetBNBeta()
{
	if(!file[3].is_open())
	{
		std::cout<<"can not found the Beta file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[3].eof())
	{
		file[3] >> temp;
		if(temp != '\n'){
			m_dBeta[index%350] = temp;
			index ++; 
		}
	}
}

void CEstimateCollisionNN::SetBNMean()
{
	if(!file[4].is_open())
	{
		std::cout<<"can not found the Mean file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[4].eof())
	{
		file[4] >> temp;
		if(temp != '\n'){
			m_dMean[index%350] = temp;
			index ++; 
		}
	}
}

void CEstimateCollisionNN::SetBNVariance()
{
	if(!file[5].is_open())
	{
		std::cout<<"can not found the Variance file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[5].eof())
	{
		file[5] >> temp;
		if(temp != '\n'){
			m_dVariance[index%350] = temp;
			index ++; 
		}
	}
}

void CEstimateCollisionNN::SetInputNormalMinMax()
{
	if(!file[6].is_open())
	{
		std::cout<<"can not found the Input Normalize file"<<std::endl;
	}
	int index = 0;
	float temp;
	while(!file[6].eof())
	{
		file[6] >> temp;
		if (index <30)
			m_dInputNormalMax[index%30] = temp;
		else if (index<60)
			m_dInputNormalMin[index%30] = temp;
		else if (index==60)
			m_dInputNormalMax[30] = temp;
		else if (index==61)
			m_dInputNormalMin[30] = temp;
		index ++;
	}
}