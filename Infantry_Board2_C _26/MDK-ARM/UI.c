#include "UI.h"
#include "typedef.h"

uint8_t UI_data[128];
uint8_t figure_data[110];

void UI_Paint(uint16_t robot_id,int n);



void UI_Paint(uint16_t robot_id,int n)
{
	uint16_t cmd_id;
	if( n == 1)
	{
		cmd_id = 0x0101;
	}
	else if( n == 2)
	{
		cmd_id = 0x0102;
	}
	else if( n == 5)
	{
		cmd_id = 0x0103;
	}
	else if( n == 7)
	{
		cmd_id = 0x0104;
	}

	memcpy(&UI_data[0],&cmd_id,2);
	memcpy(&UI_data[2],&robot_id,2);
	memcpy(&UI_data[4],&robot_id,2);
	for(int i=0;i<n;i++)
	{
		memcpy(&UI_data[4+i*15],&figure_data[i*15],15);
	}
}
