/*
 * ReachSpace.h
 *
 *  Created on: 30 Jun 2011
 *      Author: icub
 */

//const unsigned int NUM_POSES = 28;
////			0			1			2			3			4			5		6		7			8		9			10			11			12			13			14			15
//const double ARM_POSES[28][16]={
//		{	10.05	,	43.73	,	-15.38	,	16.91	,	19.71	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//0
//		{	5.66	,	36.7	,	-25.4	,	23.67	,	35.56	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//1
//		{	5.75	,	31.43	,	-25.93	,	34.57	,	40.85	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//2
//		{	-0.58	,	24.75	,	-25.75	,	39.14	,	49.68	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//3
//		{	-7.26	,	17.55	,	-26.63	,	41.78	,	59		,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//4
//		{	-17.19	,	27.57	,	1.05	,	32.64	,	41.75	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//5
//		{	-31.86	,	16.58	,	-2.91	,	19.72	,	53.55	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//6
//		{	-27.73	,	20.98	,	21.79	,	33.95	,	43.37	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//7
//		{	-10.16	,	23.61	,	6.14	,	48.89	,	45.74	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//8
//		{	2.93	,	35.56	,	7.29	,	56.45	,	35.45	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//9
//		{	7.94	,	31.7	,	-4.75	,	54.87	,	42.55	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//10
//		{	10.32	,	14.74	,	-29.88	,	56.45	,	54.29	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//11
//		{	-24.48	,	12.01	,	19.41	,	44.15	,	50.9	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//12
//		{	-10.51	,	25.63	,	19.76	,	60.58	,	41.25	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//13
//		{	3.29	,	37.32	,	16.25	,	69.1	,	33.87	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//14
//		{	-16.75	,	17.2	,	28.2	,	58.73	,	49.08	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//15
//		{	-3.57	,	32.14	,	26.44	,	72.79	,	40		,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//16
//		{	10.00	,	47.25	,	23.72	,	82.64	,	31.88	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//17
//		{	0.91	,	33.45	,	29.25	,	80.26	,	46.16	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//18
//		{	-12.62	,	16.76	,	32.42	,	64.8	,	51.71	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//19
//		{	-27.29	,	10.08	,	39.36	,	50.91	,	51.37	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//20
//		{	-10.25	,	23.44	,	43.05	,	77.54	,	48.77	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//21
//		{	-1.11	,	34.86	,	40.59	,	86.06	,	46.37	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//22
//		{	9.7		,	49.01	,	37.25	,	95.99	,	41.27	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//23
//		{	2.67	,	34.16	,	45.16	,	92.3	,	53.99	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//24
//		{	-8.49	,	20.89	,	47.27	,	79.3	,	54.71	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//25
//		{	-18.95	,	7.62	,	50.08	,	63.66	,	52.95	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//26
//		{	9.44	,	40.66	,	12.65	,	68.31	,	35.41	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//27
//};


const unsigned int NUM_POSES = 21;
//			0			1			2			3			4			5		6		7			8		9			10			11			12			13			14			15
const double ARM_POSES[NUM_POSES][16]={
		{	-36.26	,	14.82	,	54.12	,	56.36	,	51.98	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//0
		{	-26.04	,	44.26	,	36.72	,	60.4	,	36.96	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//1
		{	-39.87	,	42.07	,	25.83	,	15.00	,	23.28	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//2
		{	-16.96	,	23.79	,	43.58	,	74.38	,	60.22	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//3
//		{	-24.2	,	10.96	,	50.17	,	63.48	,	60.64	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//4
		{	-33.86	,	12.63	,	34.18	,	46.96	,	60.3	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//5
		{	-21.82	,	23.17	,	35.58	,	64.71	,	60.63	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//6
		{	-11.54	,	31.78	,	37.16	,	78.95	,	60.11	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//7
//		{	0.44	,	41.54	,	38.48	,	92.3	,	60.43	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//8 - out of view
		{	-6.09	,	41.98	,	32.51	,	82.55	,	56.38	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//9
		{	-17.34	,	31.26	,	32.07	,	68.66	,	57.62	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//10
		{	-30.79	,	19.04	,	30.75	,	48.89	,	62.09	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//11
//		{	-46.96	,	4.98	,	18.01	,	22.09	,	60.9	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//12 - joint 1 too close to body
		{	-47.31	,	10.00	,	17.57	,	20.95	,	60.87	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//13
		{	-25.6	,	23.96	,	21.08	,	51.00	,	57.32	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//14
		{	-7.24	,	40.75	,	22.75	,	74.11	,	50.06	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//15
		{	-25.43	,	26.16	,	15.63	,	48.54	,	57.1	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//16
		{	-43.73	,	14.3	,	7.29	,	21.83	,	60.39	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//17
		{	-21.05	,	31.96	,	12.3	,	46.52	,	48.05	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//18
		{	-3.69	,	46.63	,	13.35	,	63.57	,	37.36	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//19
		{	-5.13	,	48.83	,	11.07	,	54.87	,	28.09	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//20
		{	-29.38	,	30.11	,	5.62	,	33.25	,	52.13	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//21
		{	-13.74	,	44.26	,	9.31	,	40.72	,	27.17	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//22
		{	-21.05	,	46.28	,	-3.43	,	15.00	,	40.35	,	-30	,	0	,	55.53	,	0	,	66.84	,	32.43	,	5.73	,	4.67	,	5.27	,	5.24	,	5.15	},	//23
};


