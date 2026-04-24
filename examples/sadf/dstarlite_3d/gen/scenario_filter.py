#!/usr/bin/env python3
# Reads the introspection XML and filters active signals per scenario
# from the FORSYDE_SELF_REPORTING named pipe, then regenerates the DOT graph.
# Only regenerates when token rates actually change (avoids redundant updates).
import subprocess
import xml.etree.ElementTree as ET
import ast
import copy
import argparse

F2DOT = '/home/mohammad/OLD-HOME/f2dot3/f2dot'

parser = argparse.ArgumentParser(
    description='Filter introspection XML per scenario from self-reporting pipe.')
parser.add_argument('introspection_file', help='The introspection XML file (e.g. top.xml)')
parser.add_argument('self_reporting_file', help='The self-reporting named pipe (e.g. self_report)')
parser.add_argument('output_file', help='The output XML file (e.g. top_scenario.xml)')
args = parser.parse_args()

tree = ET.parse(args.introspection_file)

# Write initial full graph
with open(args.output_file, 'w') as f:
    f.write(ET.tostring(tree.getroot(), encoding='utf8').decode('utf8'))
subprocess.call(['python3', F2DOT, args.output_file])

# Track last seen rates per kernel to avoid redundant regeneration
last_state = {}   # kernel_name -> (itoks, otoks)

self_reporting_file = open(args.self_reporting_file, 'r')

for line in self_reporting_file:
    tokens = line.strip().split('  ')
    if tokens[0] != 'kernelMN':
        continue
    kernel_name = tokens[1]
    itoks = ast.literal_eval(tokens[3])
    otoks = ast.literal_eval(tokens[4])

    # Skip if nothing changed for this kernel
    key = (tuple(itoks), tuple(otoks))
    if last_state.get(kernel_name) == key:
        continue
    last_state[kernel_name] = key
    print(f"Scenario change — {kernel_name}: itoks={itoks} otoks={otoks}")

    itoks_with_ctrl = [1] + itoks   # account for the kernel control port

    tree_copy = copy.deepcopy(tree.getroot())
    delsigsset = set()
    for idx, signal in enumerate(tree_copy.findall("signal[@target='{}']".format(kernel_name))):
        if itoks_with_ctrl[idx] == 0:
            delsigsset.add(signal)
    for idx, signal in enumerate(tree_copy.findall("signal[@source='{}']".format(kernel_name))):
        if otoks[idx] == 0:
            delsigsset.add(signal)
    for signal in delsigsset:
        tree_copy.remove(signal)

    # Remove kernel if only its control port signal remains with no outputs
    if (len(tree_copy.findall("signal[@target='{}']".format(kernel_name))) == 1 and
            len(tree_copy.findall("signal[@source='{}']".format(kernel_name))) == 0):
        tree_copy.remove(tree_copy.find("leaf_process[@name='{}']".format(kernel_name)))
        tree_copy.remove(tree_copy.find("signal[@target='{}']".format(kernel_name)))

    with open(args.output_file, 'w') as f:
        f.write(ET.tostring(tree_copy, encoding='utf8').decode('utf8'))
    subprocess.call(['python3', F2DOT, args.output_file])

self_reporting_file.close()
print("Simulation ended — scenario_filter done.")
