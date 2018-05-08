import sys,os,struct,argparse,platform,re,soundfile,math,numpy,intelhex

def read_wav(input, options, print_help=False):
    if print_help:
        return 'Reading .wav file'
        
    print 'Loading wav file \'{0}\''.format(input)

    data, samplerate = soundfile.read(input)

    metadata = 'File:{0}, Rate:{1} Hz'.format(input, samplerate)

    return None, data, metadata
    
def read_hex(input, options, print_help=False):
    if print_help:
        return 'Reading .hex file'
        
    return None, intelhex.IntelHex(input), None
    
def write_c(data_type, input, output, options, metadata=None, print_help=False):
    if print_help:
        return 'Writing .c file containing array of data'
        
    print 'Writing c file \'{0}\''.format(output)
    
    if re.match(r'.*--width=(8|16|24|32).*', options):
        width = int(re.findall(r'.*--width=(8|16|24|32).*', options)[0])
    else:
        width = 16
        
    print 'Using width: {0} bits'.format(width)
    
    if re.match(r'.*--endianness=big.*', options):
        print 'Using big endian'
        fmt_endian = '>'
        fmt_text   = 'big'
    else:
        print 'Using little endian'
        fmt_endian = '<'
        fmt_text   = 'little'
        
    if type(input[0]) == numpy.ndarray:
        # More than 1 channel: flatten all frames into one list
        channels = len(input[0])
        input_flattened = []
        for frame in input:
            for channel in frame:
                input_flattened.append(channel)
        input = input_flattened
    else:
        channels = 1
        
    print 'Found {0} audio channels'.format(channels)
    
    frames = []
    max_val = math.pow(2, (width-1))
    for f in input:
        frame = struct.pack(fmt_endian + 'i', int(max_val * f) << (32 - width))
        if fmt_endian == '>':
            frames.extend(frame[0:(width/8)])
        else:
            frames.extend(frame[(4 - ((width/8))):4])

    s  = '#include <stdint.h>\n\n'
    if metadata is not None:
        s += '// Metadata from original file: {0}\n'.format(metadata)
        s += '// Array contains {0} audio samples as signed {1} endian {2}-bit integers\n'.format(len(input), fmt_text, width)
    s += 'const uint8_t {0}[{1}] = '.format(re.sub(r'\..*$', '', output), len(frames))
    s += '{'
    s += ''.join(map(lambda x: '0x{0:X}, '.format(struct.unpack(fmt_endian + 'B', x)[0]), frames))
    s += '};\n'
    
    fout = open(output, 'wb')
    fout.write(s)
    fout.close()
    
def write_raw(data_type, input, output, options, metadata=None, print_help=False):
    if print_help:
        return 'Writing .raw file containing data array.'
        
    print 'Writing raw file \'{0}\''.format(output)
    
    if data_type == 'int8_t':
        fmt = 'b'
    elif data_type == 'int16_t':
        fmt = '<h'
    elif data_type == 'int32_t':
        fmt = '<i'
    else:
        print 'ERROR (write_raw): Unsupported data type: {0}'.format(data_type)
        exit(-1)
        
    s = ''

    for i in input:
        s += struct.pack(fmt, i)
    
    fout = open(output, 'wb')
    fout.write(s)
    fout.close()

def write_hrhex(data_type, input, output, options, metadata=None, print_help=False):
    if print_help:
        return 'Writing .txt file containing human-readable hex-type formatted data array.'
        
    start_addr = 0
    if re.match(r'.*--start-address=([x0-9]+).*', options):
        start_addr = (re.findall(r'.*--start-address=([x0-9]+).*', options)[0])
        if 'x' in start_addr:
            start_addr = int(start_addr, 16)
        else:
            start_addr = int(start_addr)

    for segment in input.segments():
        if (segment[1] - start_addr) > 0x200000 and not re.match(r'.*--addr-override.*', options):
            print 'Ignoring segment 0x{0:X} - 0x{1:X}. Use --addr-override option to override'.format(segment[0], segment[1])
        else:
            end_addr = segment[1]
        
    print 'Using start address: 0x{0:08x}'.format(start_addr)
    print 'Using end address:   0x{0:08x}'.format(end_addr)
    
    s = ''
    for i in range(start_addr, end_addr, 16):
        s += '{0:08X}:'.format(i)
        for j in range(i, i+16):
            s += ' {0:02X}'.format(input[j])
        s += '\n'
    if  int(end_addr/16.0) != end_addr/16.0:
        s += '{0:08X}:'.format(int(end_addr/16.0)*16)
        for j in range(int(end_addr/16.0)*16, end_addr):
            s += ' {0:02X}'.format(input[j])
        s += '\n'
        
    open(output, 'wb').write(s)
        
        
    
supported_readers = {}
supported_readers['wav'] = read_wav
supported_readers['hex'] = read_hex

supported_writers = {}
supported_writers['c']     = write_c
supported_writers['raw']   = write_raw
supported_writers['hrhex'] = write_hrhex

def print_helper_strings():
    print 'Supported reader commands:'
    for key in supported_readers.keys():
        print '\t{0} - {1}'.format(key, supported_readers[key](None, None, print_help=True))
        
    print 'Supported writer commands:'
    for key in supported_writers.keys():
        print '\t{0} - {1}'.format(key, supported_writers[key](None, None, None, None, print_help=True))

if __name__ == '__main__':
    if 'help' in ''.join(sys.argv):
        print_helper_strings()
        exit(0)
            
    if len(sys.argv) < 4:
        print 'usage: {0} operation input output [OPTIONS]'.format(sys.argv[0])
        exit(-1)
        
    operation = sys.argv[1]
    input     = sys.argv[2]
    output    = sys.argv[3]
    if len(sys.argv) > 4:
        options   = sys.argv[4]
    else:
        options = ''
        
    print options
    
    read_operation, write_operation = operation.split('2')
    
    if read_operation in supported_readers.keys() and write_operation in supported_writers.keys():
        data_type, data, metadata = supported_readers[read_operation](input, options)
        supported_writers[write_operation](data_type, data, output, options, metadata)
    else:
        print 'Unsupported operation.'
        print 'Supported readers:'
        for reader in supported_readers.keys():
            print '\t \'{0}\''.format(reader)
        print 'Supported writers:'
        for writer in supported_writers.keys():
            print '\t \'{0}\''.format(writer)
     