/**
 * omap-abe-tlv320aic3x.h - ASoC machine driver OMAP4+ devices using
 * tlv320aic3x codec, header.
 *
 * Copyright (c) 2013 Phytec
 * All rights reserved.
 * Author: Steve Schefter <steve@scheftech.com>
 *
 * based on omap-abe-twl6040.h
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _OMAP_ABE_TLV320AIC3X_H_
#define _OMAP_ABE_TLV320AIC3X_H_

struct omap_abe_tlv320aic3x_data {
	char		*card_name;
	struct clk	*clk;
	int		mclk_freq;	/* MCLK frequency speed */
};

#endif /* _OMAP_ABE_TLV320AIC3X_H_ */
