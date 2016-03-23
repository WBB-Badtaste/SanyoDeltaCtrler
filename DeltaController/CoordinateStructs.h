#pragma once

static const uint32_t NUM_COORD_TYPES(6);

//����ϵ����
typedef enum CoordType
{
	BASE_COORD		= NUM_COORD_TYPES - 6,
	KIN_COORD		= NUM_COORD_TYPES - 5,
	CAMERA_COORD	= NUM_COORD_TYPES - 4,
	BELT_COORD		= NUM_COORD_TYPES - 3,
	TARGET_COORD	= NUM_COORD_TYPES - 2,
	JOIN_COORD		= NUM_COORD_TYPES - 1//ע��ת������
}COORD_TYPE;

//����ϵ�ṹ��
typedef struct RocksCoordinate
{
	ROCKS_E3_VECTOR position;
	COORD_TYPE type;
	double cuEncoderValue;
}ROCKS_COORD;

//ת������ṹ��
typedef struct TransfMatrix
{
	ROCKS_E3_VECTOR t;
	ROCKS_E3_VECTOR r;
	double zoom;
//	ROCKS_E3_VECTOR c;
}TRANSF_MATRIX;