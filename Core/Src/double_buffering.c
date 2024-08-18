/*
 * double_buffering.c
 *
 * Manages flags and pointers for double buffering
 *
 *  Created on: Aug 18, 2024
 *      Author: mirco
 */

#include "double_buffering.h"

void Double_Buffer_Init(Double_Buffer_t *hbuffer)
{
	hbuffer->buffer_current = hbuffer->buffer1;
	hbuffer->write_index = 0;
	hbuffer->flag_save_buffer_1 = 0;
	hbuffer->flag_save_buffer_2 = 0;
	hbuffer->flag_overflow = 0;
}

void Double_Buffer_Increment(Double_Buffer_t *hbuffer)
{
	// If next index would overflow
	if (++hbuffer->write_index >= hbuffer->buffer_len)
	{
		// Switch buffers
		hbuffer->write_index = 0;
		// If buffer_2 has been filled
		if (hbuffer->buffer_current == hbuffer->buffer2)
		{
			// If buffer_1 has been emptied
			if (!hbuffer->flag_save_buffer_1)
			{
				// buffer_2 full
				hbuffer->flag_save_buffer_2 = 1;
				// Reset buffer_1 to use
				hbuffer->buffer_current = hbuffer->buffer1;
			}
			else
			{
				// Both buffers are full, flag as overflow
				hbuffer->flag_overflow = 1;
			}
		}
		// If buffer_1 has been filled
		else
		{
			// If buffer_2 has been emptied
			if (!hbuffer->flag_save_buffer_2)
			{
				// buffer_1 full
				hbuffer->flag_save_buffer_1 = 1;
				// Reset buffer_2 to use
				hbuffer->buffer_current = hbuffer->buffer2;
			}
			else
			{
				// Both buffers are full, flag as overflow
				hbuffer->flag_overflow = 1;
			}
		}
	}
}
