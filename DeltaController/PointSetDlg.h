#pragma once


// CPointSetDlg dialog

class CPointSetDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CPointSetDlg)

public:
	CPointSetDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CPointSetDlg();

// Dialog Data
	enum { IDD = IDD_DIALOG_POINT_SETTING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
