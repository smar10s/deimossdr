#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# bd_graph.py — Parse the deimos block design (system_bd.tcl) into a cell/edge
# graph. Extracted from bd_parity.py; the block design is the single authored
# source of truth for the receiver pipeline (see DECISIONS.md D19).

import re

# create_bd_cell -type module -reference <module> <cellname>
_CELL_RE = re.compile(r'create_bd_cell -type module -reference (\S+) (\S+)')

# ad_ip_instance xlconstant <cell> / ad_ip_parameter <cell> CONFIG.CONST_* <n>
_CONST_RE = re.compile(
    r'ad_ip_parameter\s+(\S+)\s+CONFIG\.(CONST_WIDTH|CONST_VAL)\s+(\d+)')


def parse_bd(path):
    """Return (module_cells, edges).

    module_cells: {cellname: module} from create_bd_cell -type module
    edges: list of (src_endpoint, dst_endpoint); endpoint = (cell, port)
    with port None for bare nets. Parses ad_connect and both
    connect_bd_net forms (including line-continued -net form).
    """
    src = open(path).read()
    cells = {}
    for m in _CELL_RE.finditer(src):
        cells[m.group(2)] = m.group(1)
    edges = []

    def ep(x):
        x = x.strip()
        if "/" in x:
            cell, port = x.split("/", 1)
            return (cell, port)
        return (x, None)

    for m in re.finditer(r'^\s*ad_connect\s+(\S+)\s+(\S+)\s*(?:#.*)?$', src, re.M):
        edges.append((ep(m.group(1)), ep(m.group(2))))

    # connect_bd_net [get_bd_pins A] [get_bd_pins B]
    for m in re.finditer(
            r'connect_bd_net\s+\[get_bd_pins\s+(\S+)\]\s+\[get_bd_pins\s+(\S+)\]', src):
        edges.append((ep(m.group(1)), ep(m.group(2))))

    # connect_bd_net -net [get_bd_nets -of_objects [get_bd_pins A]] \
    #     [get_bd_pins B]   (line-continued)
    for m in re.finditer(
            r'connect_bd_net\s+-net\s+\[get_bd_nets\s+-of_objects\s+\[get_bd_pins\s+(\S+)\]\]\s*\\?\s*'
            r'\[get_bd_pins\s+(\S+)\]', src):
        edges.append((ep(m.group(1)), ep(m.group(2))))

    return cells, edges


def const_values(src):
    """{cell: (width, value)} from xlconstant ad_ip_parameter lines."""
    out = {}
    for cell, key, val in _CONST_RE.findall(src):
        width, value = out.get(cell, (0, 0))
        if key == "CONST_WIDTH":
            width = int(val)
        else:
            value = int(val)
        out[cell] = (width, value)
    return out
