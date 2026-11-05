/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2022-2023 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/05.
 *
 *----------------------------------------------------------------------
 */

/*
 *	i2c_cnf_sysdep.h 
 *	I2C device configuration file
 *		for RP2040
 */
#ifndef	__DEV_I2C_CNF_RP2040_H__
#define	__DEV_I2C_CNF_RP2040_H__

/* Device initialization */
#define	DEVCNF_I2C_RESET	TRUE	// Reset I2C module
#define	DEVCNF_I2C_SETPINFUNC	TRUE	// Set I/O pin function

/* I/O pins, claimed when the driver starts (DEVCNF_USE_IIC = 1).
 * Each must be a pin that has that I2C function (RP2040 datasheet,
 * GPIO function table), and must be recorded in TEAM.md.
 *   I2C0: SDA on GP0/4/8/12/16/20/28, SCL on GP1/5/9/13/17/21
 *   I2C1: SDA on GP2/6/10/14/18/22/26, SCL on GP3/7/11/15/19/27
 * Robo Pico: I2C0 = GP4/GP5 (Grove 3), I2C1 = GP2/GP3 (Grove 2, Maker port).
 * GP8/GP9 are its motor 1 pins. */
#define	DEVCNF_I2C0_SDA_PIN	4
#define	DEVCNF_I2C0_SCL_PIN	5
#define	DEVCNF_I2C1_SDA_PIN	6
#define	DEVCNF_I2C1_SCL_PIN	7

/* Register initial value */

// Interrupt priority
#define	DEVCNF_I2C0_INTPRI	2
#define	DEVCNF_I2C1_INTPRI	2

/* Communication timeout time */
#define	DEVCNF_I2C0_TMO		1000
#define	DEVCNF_I2C1_TMO		1000

#endif		/* __DEV_I2C_CNF_RP2040_H__ */
