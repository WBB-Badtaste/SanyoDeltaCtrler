
// DeltaControllerDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "DeltaController.h"
#include "DeltaControllerDlg.h"
#include "afxdialogex.h"

#include "Defines.h"

#include "MotionStateMach.h"

#ifdef _DEBUGF
#define new DEBUG_NEW
#endif




// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{

}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CDeltaControllerDlg 对话框




CDeltaControllerDlg::CDeltaControllerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDeltaControllerDlg::IDD, pParent)
	, m_motion_par_x(0)
	, m_motion_par_y(0)
	, m_motion_par_z(0)
	, m_motion_par_vel(0)
	, m_motion_par_direc(0)
	, m_tTime(0.0)
	//, m_msm(this->m_hWnd)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_tTime = 0.0;
	m_turn_angle = 0.0;


	//  m_dRobotPos_x = 0.0;
	//  m_dRobotPos_y = 0.0;
	//  m_dRobotPos_z = 0.0;
}


CDeltaControllerDlg::~CDeltaControllerDlg()
{

}


void CDeltaControllerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_listBox);
	DDX_Text(pDX, IDC_EDIT1, m_motion_par_x);
	DDX_Text(pDX, IDC_EDIT2, m_motion_par_y);
	DDX_Text(pDX, IDC_EDIT3, m_motion_par_z);
	DDX_Text(pDX, IDC_EDIT4, m_motion_par_vel);
	DDX_Radio(pDX, IDC_RADIO1, m_motion_par_direc);
// 	DDX_Control(pDX, IDC_COMB_MODEL_SHAP, m_combctr_markShape);
// 	DDX_Text(pDX, IDC_EDIT_RESULT, m_edit_locate_result);
// 	DDX_Text(pDX, IDC_EDIT_COSTTIME, m_edit_match_time);
	//  DDX_Text(pDX, IDC_EDIT5, m_tTime);
	//	DDX_Text(pDX, IDC_EDIT5, m_tTime);
	//	DDV_MinMaxDouble(pDX, m_tTime, 0.0, 100);
	DDX_Text(pDX, IDC_EDIT6, m_turn_angle);
	DDV_MinMaxDouble(pDX, m_turn_angle, -360, 360);
	//  DDX_Text(pDX, IDC_EDIT_POS_X, m_dRobotPos_x);
	//  DDX_Text(pDX, IDC_EDIT_POS_Y, m_dRobotPos_y);
	//  DDX_Text(pDX, IDC_EDIT_POS_Z, m_dRobotPos_z);
	DDX_Control(pDX, IDC_EDIT_POS_X, m_bobotPos_x);
	DDX_Control(pDX, IDC_EDIT_POS_Y, m_bobotPos_y);
	DDX_Control(pDX, IDC_EDIT_POS_Z, m_bobotPos_z);
}


BEGIN_MESSAGE_MAP(CDeltaControllerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON6, &CDeltaControllerDlg::OnBnClickedDoor)
	ON_BN_CLICKED(IDC_BUTTON7, &CDeltaControllerDlg::OnBnClickedPtp)
	ON_BN_CLICKED(IDC_BUTTON3, &CDeltaControllerDlg::OnBnClickedInit)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_CHECK1, &CDeltaControllerDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BUTTON8, &CDeltaControllerDlg::OnBnClickedHome)
	ON_BN_CLICKED(IDC_BUTTON2, &CDeltaControllerDlg::OnBnClickedBrake)
//	ON_BN_CLICKED(IDC_BUTTON5, &CDeltaControllerDlg::OnBnClickedShowPos)
	ON_BN_CLICKED(IDC_BUTTON10, &CDeltaControllerDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON13, &CDeltaControllerDlg::OnBnClickedMove1)
	ON_BN_CLICKED(IDC_BUTTON11, &CDeltaControllerDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CDeltaControllerDlg::OnBnClickedMove10)
	ON_BN_CLICKED(IDC_BUTTON1, &CDeltaControllerDlg::OnBnClickedPump)
	ON_BN_CLICKED(IDC_BUTTON4, &CDeltaControllerDlg::OnBnClickedSwitch)
	ON_BN_CLICKED(IDC_BUTTON9, &CDeltaControllerDlg::OnBnClickedCirlce)
	ON_BN_CLICKED(IDC_BUTTON14, &CDeltaControllerDlg::OnBnClickedCatch)
	ON_BN_CLICKED(IDC_BUTTON16, &CDeltaControllerDlg::OnBnClickedBelt)
// 	ON_BN_CLICKED(IDC_BTN_OPENCAMERA, &CDeltaControllerDlg::OnBnClickedBtnOpencamera)
// 	ON_WM_TIMER()
// 	ON_BN_CLICKED(IDC_BTN_ATTRIBUTE, &CDeltaControllerDlg::OnBnClickedBtnAttribute)
// 	ON_BN_CLICKED(IDC_CHECK_SEARCHAREA, &CDeltaControllerDlg::OnBnClickedCheckSearcharea)
// 	ON_BN_CLICKED(IDC_BTN_LEARN_MODEL, &CDeltaControllerDlg::OnBnClickedBtnLearnModel)
// 	ON_BN_CLICKED(IDC_BTN_MATCH_MODEL, &CDeltaControllerDlg::OnBnClickedBtnMatchModel)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONDBLCLK()

	//----JoMar,20160116
//	ON_MESSAGE(WM_MATCH_MODEL, OnMatchModel) 
	ON_MESSAGE(WM_HANDLE_NYCESTATUS, OnHanderNyceStatus) 
	ON_MESSAGE(WM_UPDATE_ROBOT_POS, OnUpdateRobotPos) 
	ON_EN_CHANGE(IDC_EDIT5, &CDeltaControllerDlg::OnEnChangeEdit5)
//	ON_BN_CLICKED(IDC_BUTTON19, &CDeltaControllerDlg::OnBnClickedButton19)
	ON_BN_CLICKED(IDC_BUTTON_TURN, &CDeltaControllerDlg::OnBnClickedButtonTurn)
END_MESSAGE_MAP()


// CDeltaControllerDlg 消息处理程序
BOOL CDeltaControllerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	

	m_pMsm = new CMotionStateMach(this->m_hWnd);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}


void CDeltaControllerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}


// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CDeltaControllerDlg::OnPaint()
{
	

	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CDeltaControllerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CDeltaControllerDlg::OnBnClickedDoor()//门型运动
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);

	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if(!m_pMsm->SwitchToDoorState(m_motion_par_vel))
		PrintStr("Robot is not ready.");
}


void CDeltaControllerDlg::OnBnClickedPtp()//移动到指定点
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);

	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if (m_motion_par_x == 0 || m_motion_par_y == 0 || m_motion_par_z == 0)
	{
		PrintStr("Please input a position.");
		return;
	}


	if(!m_pMsm->SwitchToPtpState(m_motion_par_x, m_motion_par_y, m_motion_par_z, m_motion_par_vel))
		PrintStr("Robot is not ready.");

}

void CDeltaControllerDlg::OnBnClickedInit()//初始化
{
	// TODO: Add your control notification handler code here
	if(!m_pMsm->SwitchToInitState())
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: Add your message handler code here

	delete m_pMsm;

}


void CDeltaControllerDlg::OnBnClickedCheck1()
{
	// TODO: Add your control notification handler code here
/*	NYCE_STATUS nyceStatus(NYCE_OK);*/
}

void CDeltaControllerDlg::OnBnClickedHome()//回零位
{
	// TODO: Add your control notification handler code here
	if(!m_pMsm->SwitchToHomeState())
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnBnClickedBrake()//刹车控制
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT0;
// 
// 	uint32_t ioStatus1(0), ioStatus2(0), ioStatus3(0);
// 	io.digIONr = NYCE_DIGOUT0;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus1);
// 	io.digIONr = NYCE_DIGOUT1;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus2);
// 	io.digIONr = NYCE_DIGOUT2;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus3);
// 
// 	if (ioStatus1 && ioStatus2 && ioStatus3)
// 	{
// 		io.digIONr = NYCE_DIGOUT0;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT1;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT2;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	}
// 	else
// 	{
// 		io.digIONr = NYCE_DIGOUT0;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT1;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT2;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 	}
// 
// 	StatusHandle(nyceStatus);
}





void CDeltaControllerDlg::OnBnClickedButton10()//-1
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(-1, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedMove1()//+1
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(1, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedButton11()//-10
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(-10, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedMove10()//+10
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(10, m_motion_par_direc);
}

void CDeltaControllerDlg::OnBnClickedPump()//控制真空泵
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT0;
// 	io.digIONr = NYCE_DIGOUT3;
// 
// 	uint32_t ioStatus(0);
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);
// 
// 	if (ioStatus)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedSwitch()//控制电磁阀
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT3;
// 	io.digIONr = NYCE_DIGOUT2;
// 
// 	uint32_t ioStatus(0);
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);
// 
// 	if (ioStatus)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedCirlce()//圆形轨迹
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if(!m_pMsm->SwitchToCircState(m_motion_par_vel))
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnBnClickedCatch()
{

	if(!m_pMsm->SwitchToCatchState())
	{
		PrintStr("Robot is not ready.");
	}

}


void CDeltaControllerDlg::OnBnClickedBelt()//传送带控制
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	SAC_STATE state;
// 	SAC_SPG_STATE spgState;
// 	SacReadState(beltId[0], &state, &spgState);
// 
// 	SAC_JOG_PARS jogPars;
// 	jogPars.velocity = BELT_VEL * BELT_BASE_RATE;
// 	jogPars.acceleration = jogPars.velocity * 10;
// 	jogPars.jerk = jogPars.velocity * 100;
// 
// 	if (state == SAC_READY)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStartJog(beltId[0], &jogPars);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStopJog(beltId[0], &jogPars);
// 
// 	StatusHandle(nyceStatus);
}



LRESULT CDeltaControllerDlg::OnUpdateRobotPos(WPARAM wParam, LPARAM lParam)
{

	double *pos = (double *)lParam;
	CString str("");

	str.Format(_T("%f"), pos[0]);
	m_bobotPos_x.SetWindowText(str);

	str.Format(_T("%f"), pos[1]);
	m_bobotPos_y.SetWindowText(str);
	
	str.Format(_T("%f"), pos[2]);
	m_bobotPos_z.SetWindowText(str);
	
	return 0;
}

LRESULT CDeltaControllerDlg::OnHanderNyceStatus(WPARAM wParam, LPARAM lParam)
{
	m_listBox.AddString(*(CString*)lParam);
	return 0;
}

void CDeltaControllerDlg::OnEnChangeEdit5()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}



void CDeltaControllerDlg::OnBnClickedButtonTurn()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
//	RocksRotateAngle(m_turn_angle);
	UpdateData(FALSE);
}


void CDeltaControllerDlg::PrintStr(const CString &str)
{
	CString string;
	SYSTEMTIME time;
	GetSystemTime(&time);
	string = "";
	string.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
	string += ": ";
	string += str;
	string += "\n";
	m_listBox.AddString(string);
}