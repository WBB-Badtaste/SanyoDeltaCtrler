
// DeltaController.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CDeltaControllerApp:
// �йش����ʵ�֣������ DeltaController.cpp
//

class CDeltaControllerApp : public CWinApp
{
public:
	CDeltaControllerApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CDeltaControllerApp theApp;