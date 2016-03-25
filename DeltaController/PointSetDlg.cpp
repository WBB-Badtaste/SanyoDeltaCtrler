// PointSetDlg.cpp : implementation file
//

#include "stdafx.h"
#include "DeltaController.h"
#include "PointSetDlg.h"
#include "afxdialogex.h"


// CPointSetDlg dialog

IMPLEMENT_DYNAMIC(CPointSetDlg, CDialogEx)

CPointSetDlg::CPointSetDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPointSetDlg::IDD, pParent)
{

}

CPointSetDlg::~CPointSetDlg()
{
}

void CPointSetDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CPointSetDlg, CDialogEx)
END_MESSAGE_MAP()


// CPointSetDlg message handlers
