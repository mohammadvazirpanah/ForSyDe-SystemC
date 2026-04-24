#!/usr/bin/env python3
# Scripts to read the introspection XML file and filter for each scenario based on the self-reporting information read from a named pipe.
import subprocess
import xml.etree.ElementTree as ET
import ast
import copy
import argparse

# Parse the command line arguments
parser = argparse.ArgumentParser(description='Filter the introspection XML file for each scenario based on the self-reporting information read from a named pipe.')
parser.add_argument('introspection_file', help='The introspection XML file')
parser.add_argument('self_reporting_file', help='The self-reporting named pipe')
parser.add_argument('output_file', help='The output file')

args = parser.parse_args()

# Read the introspection XML file
tree = ET.parse(args.introspection_file)

# Creare the base output as a copy of the introspection XML file
with open(args.output_file, 'w') as f:
    f.write(ET.tostring(tree.getroot(), encoding='utf8').decode('utf8'))
# Convert to DOT
subprocess.call(['python3', '/home/mohammad/f2dot3/f2dot', args.output_file])

# Open the self-reporting named pipe
self_reporting_file = open(args.self_reporting_file, 'r')

# Read the self-reporting information
for line in self_reporting_file:
    # Open the output file
    output_file = open(args.output_file, 'w')
    # make a copy  of the tree
    tree_copy = copy.deepcopy(tree.getroot())
    # Parse the self-reporting information
    line = line.strip().split('  ')
    if line[0] != 'kernelMN':
        continue
    kernel_name = line[1]
    print(line)
    itoks = ast.literal_eval(line[3])
    otoks = ast.literal_eval(line[4])
    itoks.insert(0,1)   # To count for the kernel control port
    # Filter the introspection XML file
    delsigsset = set()
    for idx,signal in enumerate(tree_copy.findall("signal[@target='{}']".format(kernel_name))):
        if itoks[idx] == 0:
            # tree_copy.remove(signal)
            delsigsset.add(signal)
    for idx,signal in enumerate(tree_copy.findall("signal[@source='{}']".format(kernel_name))):
        if otoks[idx] == 0:
            # tree_copy.remove(signal)
            delsigsset.add(signal)
    for signal in delsigsset:
        tree_copy.remove(signal)
    # Delete the kernel if it has no connecting signals (except for the control port)
    if len(tree_copy.findall("signal[@target='{}']".format(kernel_name))) == 1 and len(tree_copy.findall("signal[@source='{}']".format(kernel_name))) == 0:
        tree_copy.remove(tree_copy.find("leaf_process[@name='{}']".format(kernel_name)))
        # Delete the kernel's control port
        tree_copy.remove(tree_copy.find("signal[@target='{}']".format(kernel_name)))
    # Write the output XML file
    output_file.write(ET.tostring(tree_copy, encoding='utf8').decode('utf8'))
    # Close the output file
    output_file.close()
    # Convert to DOT
    subprocess.call(['python3', '/home/mohammad/f2dot3/f2dot', args.output_file])

# Close the self-reporting named pipe
self_reporting_file.close()
