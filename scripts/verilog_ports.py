#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# verilog_ports.py — Verilator XML model: module ports (dir/width/signed),
# instance map, and instance port->net links. Widths resolve through
# <basicdtype id=... left=... right=...>, which --xml-only stores separately
# from the <var dtype_id=...> references.

from collections import namedtuple
import xml.etree.ElementTree as ET

Port = namedtuple("Port", "dir width signed")


def _dtype_table(root):
    table = {}
    for d in root.iter():
        if d.get("id") and d.get("name") in ("logic", "bit", "reg", "real"):
            left, right = d.get("left"), d.get("right")
            width = abs(int(left) - int(right)) + 1 if left is not None and right is not None else 1
            table[d.get("id")] = (width, d.get("signed") == "true")
    return table


def _root(xml_path):
    return ET.parse(xml_path).getroot()


def _modules(root):
    return {m.get("name"): m for m in root.findall(".//module")}


def module_ports(xml_path):
    """{module: {port: Port}} for module port vars (input/output)."""
    root = _root(xml_path)
    dt = _dtype_table(root)
    out = {}
    for name, m in _modules(root).items():
        ports = {}
        for v in m.findall("var"):
            d = v.get("dir")
            if d in ("input", "output"):
                width, signed = dt.get(v.get("dtype_id"), (1, False))
                ports[v.get("name")] = Port(d, width, signed)
        out[name] = ports
    return out


def instances(xml_path, module):
    """{instance_name: defName} for one module."""
    m = _modules(_root(xml_path)).get(module)
    if m is None:
        raise KeyError(f"module {module!r} not in XML")
    return {i.get("name"): i.get("defName") for i in m.findall("instance")}


def instance_ports(xml_path, module):
    """{instance: {port: net_name_or_None}} for one module.

    net is the <varref> target; None when the port is tied to a <const>
    or left unconnected.
    """
    m = _modules(_root(xml_path)).get(module)
    if m is None:
        raise KeyError(f"module {module!r} not in XML")
    out = {}
    for i in m.findall("instance"):
        pmap = {}
        for p in i.findall("port"):
            vr = p.find("varref")
            pmap[p.get("name")] = vr.get("name") if vr is not None else None
        out[i.get("name")] = pmap
    return out
