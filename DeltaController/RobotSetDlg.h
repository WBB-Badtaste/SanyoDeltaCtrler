#pragma once


// CRobotSetDlg dialog

class CRobotSetDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CRobotSetDlg)

public:
	CRobotSetDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CRobotSetDlg();

// Dialog Data
	enum { IDD = IDD_DIALOG_ROBOT_SETTING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
