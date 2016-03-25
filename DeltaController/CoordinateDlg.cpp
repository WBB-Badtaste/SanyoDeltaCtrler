// CoordinateDlg.cpp : implementation file
//

#include "stdafx.h"
#include "DeltaController.h"
#include "CoordinateDlg.h"
#include "afxdialogex.h"


// CCoordinateDlg dialog

IMPLEMENT_DYNAMIC(CCoordinateDlg, CDialogEx)

CCoordinateDlg::CCoordinateDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CCoordinateDlg::IDD, pParent)
{

	m_rx = 0.0;
	m_ry = 0.0;
	m_rz = 0.0;
	m_rate = 0.0;
	m_tx = 0.0;
	m_ty = 0.0;
	m_tz = 0.0;
	m_radio = 0;
}

CCoordinateDlg::~CCoordinateDlg()
{
}

void CCoordinateDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_R_X, m_rx);
	DDX_Text(pDX, IDC_EDIT_R_Y, m_ry);
	DDX_Text(pDX, IDC_EDIT_R_Z, m_rz);
	DDX_Text(pDX, IDC_EDIT_RATE, m_rate);
	DDX_Text(pDX, IDC_EDIT_T_X, m_tx);
	DDX_Text(pDX, IDC_EDIT_T_Y, m_ty);
	DDX_Text(pDX, IDC_EDIT_T_Z, m_tz);
	//  DDX_Control(pDX, IDC_RADIO_COOR_KIN, m_radio);
	DDX_Radio(pDX, IDC_RADIO_COOR_KIN, m_radio);
}


BEGIN_MESSAGE_MAP(CCoordinateDlg, CDialogEx)
END_MESSAGE_MAP()


// CCoordinateDlg message handlers
