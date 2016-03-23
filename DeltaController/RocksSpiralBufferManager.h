#pragma once

#include <rocksapi.h>

static double (*pPosSpiralSplineBuffer)[ROCKS_MECH_MAX_DOF];
static double (*pVelSpiralSplineBuffer)[ROCKS_MECH_MAX_DOF];
static uint32_t spiralBufferSize = 0;
static BOOL bSpiralBufferAlloced = FALSE;

static void SpiralBufferManage(ROCKS_MECH *pMech, const uint32_t& nextBufferStartIndex)
{
	if (pMech->var.usedNrOfSplines == 0)//�µ�path
	{
		spiralBufferSize = 0;

		if (bSpiralBufferAlloced)//��������״ε���spiral�켣�滮���ǲ������ٻ�������
		{
			delete []pPosSpiralSplineBuffer;
			delete []pVelSpiralSplineBuffer;
		}

		pPosSpiralSplineBuffer = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();
		pVelSpiralSplineBuffer = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();

		bSpiralBufferAlloced = TRUE;
	}

	
	//����ԭ��buffer
	if ( nextBufferStartIndex > pMech->var.maxNrOfSplines)
	{
		uint32_t copySize(pMech->var.maxNrOfSplines * sizeof(double));
		pMech->var.maxNrOfSplines = nextBufferStartIndex + 512;
		uint32_t mallocSize(pMech->var.maxNrOfSplines * sizeof(double));

		double *pPosBuffer = (double *)malloc(mallocSize);
		double *pVelBuffer = (double *)malloc(mallocSize);
		ZeroMemory(pPosBuffer, mallocSize);
		ZeroMemory(pVelBuffer, mallocSize);

		memcpy(pPosBuffer, pMech->var.pPositionSplineBuffer, copySize);
		memcpy(pVelBuffer, pMech->var.pVelocitySplineBuffer, copySize);

// 		free(pMech->var.pPositionSplineBuffer);
// 		free(pMech->var.pVelocitySplineBuffer);
		pMech->var.pPositionSplineBuffer = pPosBuffer;
		pMech->var.pVelocitySplineBuffer = pVelBuffer;
	}

	//�����Զ���buffer
	if(nextBufferStartIndex > spiralBufferSize)
	{
		uint32_t copySize(spiralBufferSize * sizeof(double));
		spiralBufferSize = pMech->var.maxNrOfSplines;

		double (*pPosBuffer)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();
		double (*pVelBuffer)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();

		memcpy(pPosBuffer[0], pPosSpiralSplineBuffer[0], copySize);
		memcpy(pPosBuffer[1], pPosSpiralSplineBuffer[1], copySize);
		memcpy(pPosBuffer[2], pPosSpiralSplineBuffer[2], copySize);
		memcpy(pVelBuffer[0], pVelSpiralSplineBuffer[0], copySize);
		memcpy(pVelBuffer[1], pVelSpiralSplineBuffer[1], copySize);
		memcpy(pVelBuffer[2], pVelSpiralSplineBuffer[2], copySize);

		delete []pPosSpiralSplineBuffer;
		delete []pVelSpiralSplineBuffer;

		pPosSpiralSplineBuffer = pPosBuffer;
		pVelSpiralSplineBuffer = pVelBuffer;
	}

	bSpiralBufferAlloced = TRUE;
}