// SettingDlg.cpp : implementation file
//

#include "stdafx.h"
#include "DeltaController.h"
#include "SettingDlg.h"
#include "afxdialogex.h"


// CSettingDlg dialog

IMPLEMENT_DYNAMIC(CSettingDlg, CDialogEx)

CSettingDlg::CSettingDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CSettingDlg::IDD, pParent)
{

}

CSettingDlg::~CSettingDlg()
{
}

void CSettingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB, m_tab);
}


BEGIN_MESSAGE_MAP(CSettingDlg, CDialogEx)
	ON_NOTIFY(TCN_SELCHANGING, IDC_TAB, &CSettingDlg::OnTcnSelchangingTab)
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB, &CSettingDlg::OnTcnSelchangeTab)
END_MESSAGE_MAP()


// CSettingDlg message handlers



BOOL CSettingDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  Add extra initialization here
	m_tab.InsertItem(0,"��������");  
	m_tab.InsertItem(1,"����ϵͳ");  
	m_tab.InsertItem(2,"��������");  

	m_coordDlg.Create(IDD_DIALOG_COORDINATE,	GetDlgItem(IDC_TAB));
	m_pointDlg.Create(IDD_DIALOG_POINT_SETTING, GetDlgItem(IDC_TAB));
	m_robotDlg.Create(IDD_DIALOG_ROBOT_SETTING, GetDlgItem(IDC_TAB));

	CRect rs; 
	m_tab.GetClientRect(&rs); 
	//�����ӶԻ����ڸ������е�λ�� 
// 	rs.top		+=1; 
// 	rs.bottom	-=60; 
// 	rs.left		+=1; 
// 	rs.right	-=2; 

	//�����ӶԻ���ߴ粢�ƶ���ָ��λ�� 
	m_coordDlg.MoveWindow(&rs); 
	m_pointDlg.MoveWindow(&rs); 
	m_robotDlg.MoveWindow(&rs); 
	 
	//�ֱ��������غ���ʾ 
	m_robotDlg.ShowWindow(SW_SHOW);
	m_coordDlg.ShowWindow(SW_HIDE);
	m_pointDlg.ShowWindow(SW_HIDE);

	//����Ĭ�ϵ�ѡ� 
	m_tab.SetCurSel(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}



void CSettingDlg::OnTcnSelchangingTab(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: Add your control notification handler code here
	int nSel = m_tab.GetCurSel();
	switch(nSel) 
	{ 
	case 0: 
		m_robotDlg.ShowWindow(SW_HIDE); 
		m_coordDlg.ShowWindow(SW_SHOW); 
		m_pointDlg.ShowWindow(SW_HIDE); 
		break; 
	case 1: 
		m_robotDlg.ShowWindow(SW_SHOW); 
		m_coordDlg.ShowWindow(SW_HIDE); 
		m_pointDlg.ShowWindow(SW_HIDE); 
		break; 
	case 2: 
		m_robotDlg.ShowWindow(SW_HIDE); 
		m_coordDlg.ShowWindow(SW_HIDE); 
		m_pointDlg.ShowWindow(SW_SHOW); 
		break; 
	default: 
		break; 
	}  
	*pResult = 0;
}


void CSettingDlg::OnTcnSelchangeTab(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: Add your control notification handler code here
	*pResult = 0;
}
