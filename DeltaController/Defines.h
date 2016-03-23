#pragma once

#include "mmsystem.h"  
#include "rockstypes.h"
#include "CoordinateStructs.h"
#include "sacapi.h"
#include "nhiapi.h"


//λ����
#define NUM_AXES 3
static const char *axName[ NUM_AXES ] = { "DEF_AXIS_3", "DEF_AXIS_1", "DEF_AXIS_2"};
static SAC_AXIS axId[ NUM_AXES ];

//��ת��
#define NUM_AXES_ROTATION 1
static const char *rotationName[ NUM_AXES_ROTATION ] = {"DEF_AXIS_4"};
static SAC_AXIS rotationId[ NUM_AXES_ROTATION ];

//Ƥ����
#define NUM_AXES_BELT 1
static const char *beltaName[ NUM_AXES_BELT ] = {"DEF_AXIS_5"};
static SAC_AXIS beltId[ NUM_AXES_BELT ];

//�ڵ�
#define NUM_NODE 1
static const char *noName[ NUM_NODE ] = { "NY411x_node" };
static NHI_NODE noId[ NUM_NODE ];

//Ƥ����
#define BELT_VEL 200 //���ʹ��ٶȣ���λmm/s
#define BELT_BASE_RATE 15518.2130329563124//15548.8947763849895 //ʵ���ƶ������������ֵ�ı�ֵ
static double THRESHOLD_TIME = 0.085; //����ʱ����ֵ����λS
// double g_encoderMinRange;
// double g_encoderMaxRange;

//��ת��
#define ROTATE_ANGLE_RATE (131072 / 360)
#define ROTATE_VEL M_PI

//����λ�Ƶ�������,ʵ�ʳ���������г̵ı�ֵ
#define KIN_BASE_RATE 0.9737

//����ϵ���
static TRANSF_MATRIX *g_pTransfMatrix;

//������
#define PIXEL_BASE_RATE 3.1266775702479

//�������� 
static BOOL		g_bInitHomePos;
static ROCKS_COORD g_homePos;
static ROCKS_COORD g_placePos;
static ROCKS_COORD g_pickPos;

//�����˶�����
#define DOOR_HIGHT1 -835
#define DOOR_HIGHT2 -835
#define DOOR_PAR_X 152.5
#define DOOR_PAR_Y 0

#define READ_POS_DELAY 500//ѭ����ȡ������λ�õ�ʱ����

//�����˶�����
#define HOME_VEL 500

//Jog����
#define JOG_VEL 50

#define WM_MATCH_MODEL			WM_USER + 1
#define WM_HANDLE_NYCESTATUS	WM_USER + 2
#define WM_UPDATE_ROBOT_POS		WM_USER + 3