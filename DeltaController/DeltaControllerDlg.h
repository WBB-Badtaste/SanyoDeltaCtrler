
// DeltaControllerDlg.h : ͷ�ļ�
//

#pragma once

#include "nyceapi.h"
#include "afxwin.h"

#include "SettingDlg.h"


class CMotionStateMach;


// CDeltaControllerDlg �Ի���
class CDeltaControllerDlg : public CDialogEx
{
// ����
public:
	CDeltaControllerDlg(CWnd* pParent = NULL);	// ��׼���캯��
	~CDeltaControllerDlg();	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_DELTACONTROLLER_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��

	
// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedDoor();
	afx_msg void OnBnClickedPtp();
	afx_msg void OnBnClickedInit();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedHome();
	afx_msg void OnBnClickedBrake();
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedMove1();
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedMove10();
	afx_msg void OnBnClickedPump();
	afx_msg void OnBnClickedSwitch();
	afx_msg void OnBnClickedCirlce();
	afx_msg void OnBnClickedCatch();
	afx_msg void OnBnClickedBelt();
	afx_msg void OnBnClickedBtnBrake();
	afx_msg void OnBnClickedButtonSetting();
	afx_msg LRESULT OnHanderNyceStatus(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnUpdateRobotPos(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnUpdateBtn(WPARAM wParam, LPARAM lParam);
	CListBox m_listBox;
	CEdit m_bobotPos_x;
	CEdit m_bobotPos_y;
	CEdit m_bobotPos_z;
	double m_motion_par_x;
	double m_motion_par_y;
	double m_motion_par_z;
	double m_motion_par_vel;
	int m_motion_par_direc;
	CSettingDlg m_setDlg;
	
private:
	//״̬��
	CMotionStateMach *m_pMsm;

	void PrintStr(const CString &);
};
