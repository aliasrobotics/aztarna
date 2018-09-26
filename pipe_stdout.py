#!/usr/bin/env python
import sys

# use stdin if it's full                                                        
if not sys.stdin.isatty():
    input_stream = sys.stdin

# otherwise, read the given filename                                            
else:
    try:
        input_filename = sys.argv[1]
    except IndexError:
        message = 'need filename as first argument if stdin is not full'
        raise IndexError(message)
    else:
        input_stream = open(input_filename, 'rU')

for line in input_stream:
    print line # do something useful with each line
