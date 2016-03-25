#pragma once





//�������˶�״̬
typedef enum motionState
{
	HOME		= 0,//�����˻���
	PTP			= 1,
	CIRC		= 2,
	DOOR		= 3,
	READY		= 4,
	CATCH		= 5,
	INIT		= 6,
	CTRL_BRAKE  = 7
}MOTION_STATE;


class CMotionStateMach
{
public:
	CMotionStateMach(void);
	CMotionStateMach(HWND);
	~CMotionStateMach(void);

	bool FinlishMatch(const double &x, const double &y, const double &angle);
	bool SwitchToCatchState();
	bool SwitchToInitState();
	bool SwitchToDoorState(const double &vel);
	bool SwitchToPtpState(const double &x,const double &y,const double &z,const double &vel);
	bool SwitchToHomeState();
	bool SwitchToCircState(const double &vel);
	bool SwitchToJogState(const double &dist, const int &dire);
	bool SwitchToCtrlBrakeState();

private:

	//�����ھ��
	const HWND m_hMainWnd;

	//�����߳�
	HANDLE m_hStateThread, m_hEvST, m_hEvMove;
	HANDLE m_hReadPosThread, m_hEvRPT;
	MOTION_STATE m_status;
	static unsigned WINAPI StateThread(void *);
	static unsigned WINAPI ReadPosThread(void *);

	//��������
	const unsigned int Catch();
	const unsigned int Init();
	const unsigned int Door();
	const unsigned int Ptp();
	const unsigned int Home();
	const unsigned int Circ();
	const unsigned int CtrlBrake();

	//ROCKS״̬������
	void StatusHandler(unsigned int &);
	void SendString(const CString &str = "");

	//ƥ����
	bool m_bWaitForMatchRes;
	HANDLE m_hEvFinlishMatch;
	double m_dTargetPos_x;
	double m_dTargetPos_y;
	double m_dTargetAngle;
	//��ʼ����־
	bool m_bInit;
	//�˶��ٶ�
	double m_motionVel;
	//�ƶ�λ��
	double m_nextPos_x;
	double m_nextPos_y;
	double m_nextPos_z;
	bool   m_bPtpRelative;

	//������ʾ��Ϣ
	CString m_str;
};

