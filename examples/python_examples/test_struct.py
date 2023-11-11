#!/usr/bin/env python3
import struct

# 'i' is for integer and 'f' is for float
# The '>' indicates that the data is in big-endian format
format_string = '>if'

# Packing data
values = (1234, 56.78)
packed_data = struct.pack(format_string, *values)

# Unpacking data
unpacked_data = struct.unpack(format_string, packed_data)

print('Original values:', values)
print('Packed data   :', packed_data)
print('Unpacked data :', unpacked_data)
