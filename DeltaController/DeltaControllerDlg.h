
// DeltaControllerDlg.h : ͷ�ļ�
//

#pragma once

#include "nyceapi.h"
#include "afxwin.h"



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
	afx_msg void OnBnClickedCheck1();

public:
	afx_msg void OnBnClickedHome();
	afx_msg void OnBnClickedBrake();
//	afx_msg void OnBnClickedShowPos();
	CListBox m_listBox;
	double m_motion_par_x;
	double m_motion_par_y;
	double m_motion_par_z;
	double m_motion_par_vel;
	int m_motion_par_direc;
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedMove1();
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedMove10();
	afx_msg void OnBnClickedPump();
	afx_msg void OnBnClickedSwitch();
	afx_msg void OnBnClickedCirlce();
	afx_msg void OnBnClickedCatch();
	afx_msg void OnBnClickedBelt();
// 	afx_msg void OnBnClickedBtnOpencamera();
// 	afx_msg void OnTimer(UINT_PTR nIDEvent);
// 	afx_msg void OnBnClickedBtnAttribute();
// 	CComboBox m_combctr_markShape;
// 	afx_msg void OnBnClickedCheckSearcharea();
// 	afx_msg void OnBnClickedBtnLearnModel();
// 	afx_msg void OnBnClickedBtnMatchModel();
// 	CString m_edit_locate_result;
// 	double m_edit_match_time;
// 	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
// 	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
// 	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
// 	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
// 	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
// 	afx_msg void OnRButtonDblClk(UINT nFlags, CPoint point);
	//----JoMar,20160116
//	afx_msg LRESULT OnMatchModel(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnHanderNyceStatus(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnUpdateRobotPos(WPARAM wParam, LPARAM lParam);

public:
	afx_msg void OnEnChangeEdit5();
//	CString m_tTime;
	double m_tTime;
//	afx_msg void OnBnClickedButton19();
	afx_msg void OnBnClickedButtonTurn();
	double m_turn_angle;

private:
	//״̬��
	CMotionStateMach *m_pMsm;

	void PrintStr(const CString &);
public:
//	double m_dRobotPos_x;
//	double m_dRobotPos_y;
//	double m_dRobotPos_z;
	CEdit m_bobotPos_x;
	CEdit m_bobotPos_y;
	CEdit m_bobotPos_z;
};
