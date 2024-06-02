import argparse
import os
import sys
import binascii

samples_filename = "samples.h"



# Makes sure a string represents a valid, existing file
# This can be used with argparse as a valid argument type
def file_path(string):
    if os.path.isfile(string):
        return string
    else:
        raise FileNotFoundError(string)


# Parse arguments
def parse_args(args):
    parser = argparse.ArgumentParser(
        description=f"Converts valid audio sample kits into a max sized ATtyiny85 sketch"
                    f"Valid parameters are shown in {{braces}}\n"
                    f"Default parameters are shown in [brackets].",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument("-s0", "--sample0", type=file_path, required=True,
                        help="the first sample to use"
                        )
    
    parser.add_argument("-s1", "--sample1", type=file_path, required=True,
                        help="the second sample to use"
                        )

    parsed_args = parser.parse_args(args)

    return parsed_args


# Writes the C compatible hex dump to use in variables
def write_hex_dump(bytes_in, prefix="\t"):
    hex_dump = prefix
    byte_count = 0
    total_bytes = len(bytes_in)
    
    for byte in bytes_in:
        byte_count += 1
        hex_dump += f"0x{byte.hex()}, "
        if (byte_count % 16 == 0) and byte_count < total_bytes:
            hex_dump += "\n\t"
    
    return hex_dump

def write_sample_entry(name, index, data):
    sample_entry = f"// {name}\n"
    sample_entry += f"const uint8_t sample{index}[] PROGMEM =\n"
    sample_entry += "{\n"
    sample_entry += write_hex_dump(data)
    sample_entry += "}"
    
    return sample_entry


def main(args):
    parsed_args = parse_args(args)
    
    

def run():
    main(sys.argv[1:])


run()
