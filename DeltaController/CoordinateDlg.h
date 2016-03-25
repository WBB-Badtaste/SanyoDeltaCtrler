#pragma once


// CCoordinateDlg dialog

class CCoordinateDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CCoordinateDlg)

public:
	CCoordinateDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CCoordinateDlg();

// Dialog Data
	enum { IDD = IDD_DIALOG_COORDINATE };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	double m_rx;
	double m_ry;
	double m_rz;
	double m_rate;
	double m_tx;
	double m_ty;
	double m_tz;
//	CButton m_radio;
	int m_radio;
};
