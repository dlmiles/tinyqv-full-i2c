#
#
#
#
#
# SPDX-FileCopyrightText: Copyright 2023 Darryl Miles
# SPDX-License-Identifier: Apache2.0
#
#


#SCL_BITID		= 2	# bidi: uio_out & uio_in (old mapping)
SDA_BITID		= 3	# bidi: uio_out & uio_in
SCL_BITID		= 4	# bidi: uio_out & uio_in (new mapping, I2C0 on RP2040)

SCL_BITID_MASK = 1 << SCL_BITID
SDA_BITID_MASK = 1 << SDA_BITID

# This validate the design under test matches values here
def validate(dut) -> bool:
    return True


__all__ = [
    'SCL_BITID',
    'SDA_BITID',

    'SCL_BITID_MASK',
    'SDA_BITID_MASK',

    'validate'
]
