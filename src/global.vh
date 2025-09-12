/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// TinyQV bus interface definitions
`define DATA_READ_8BIT      (2'b00)
`define DATA_READ_16BIT     (2'b01)
`define DATA_READ_32BIT     (2'b10)
`define DATA_READ_IDLE      (2'b11)

`define DATA_WRITE_8BIT     (2'b00)
`define DATA_WRITE_16BIT    (2'b01)
`define DATA_WRITE_32BIT    (2'b10)
`define DATA_WRITE_IDLE     (2'b11)

// I2C Device project definitions
// 32bit word aligned, byte addressing
`define ADR00_DATA          (6'b000000)
`define ADR01_STAT          (6'b000100)
`define ADR02_CTRL          (6'b001000)
`define ADR03_CONF          (6'b001100)
`define ADR04_CMUX          (6'b010000)

`define DIR_TXD             (1'b0)
`define DIR_RXD             (1'b1)
