/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define MY_WIDTH	160
#define MY_HEIGHT	120

#define RAD_TO_DEG	(180.0/M_PI)
#define DEG_TO_RAD	(M_PI/180.0)

#ifdef	VGA
#	define GRAPH_WIDTH	VGA_WIDTH
#	define GRAPH_HEIGHT	VGA_HEIGHT
#else
#ifdef	QVGA
#	define GRAPH_WIDTH	QVGA_WIDTH
#	define GRAPH_HEIGHT	QVGA_HEIGHT
#else
#	define GRAPH_WIDTH	MY_WIDTH
#	define GRAPH_HEIGHT	MY_HEIGHT
#endif
#endif

#endif
