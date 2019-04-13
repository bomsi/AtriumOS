#!/usr/bin/env python

# Helper script to convert between LBA and CHS addresses.

import sys

def lba2chs(lba, cylinders=80, heads=2, sectors=18):
	if cylinders < 1 or cylinders > 1024:
		raise ValueError, '1 <= cylinders <= 1024'
	if heads < 1 or heads > 255:
		raise ValueError, '1 <= heads <= 255'
	if sectors < 1 or heads > 63:
		raise ValueError, '1 <= sectors <= 63'
	
	t, s = divmod(lba, sectors)
	s += 1
	c, h = divmod(t, heads)

	if c >= cylinders:
		raise ValueError, 'Unaddressable LBA for given geometry'

	return (c, h, s)

if len(sys.argv) != 4:
	raise ValueError, 'Arguments: (count, first, base_address)'

number_of_lbas_to_read = int(sys.argv[1]) # e.g. 17
first_lba_to_read = int(sys.argv[2]) # e.g. 1
base_address = int(sys.argv[3], 16) # e.g. 0x007e00

bytes_per_sector = 512
last_lba_to_read = first_lba_to_read + number_of_lbas_to_read
bytes = 0

for i in range(first_lba_to_read, last_lba_to_read):
	(c, h, s) = lba2chs(i)
	print('%s/%s: (%s,%s,%s) @ 0x%0.6x' % 
		(i + 1, last_lba_to_read, c, h, s, base_address + bytes))
	bytes += bytes_per_sector

print('Sectors written into 0x%0.6x ~ 0x%0.6x (%s bytes)' %
	(base_address, base_address + bytes, bytes))


