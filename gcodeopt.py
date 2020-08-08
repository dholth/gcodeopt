#!/usr/bin/env python
"""
Optimize gcode with continuous G01 runs separated by G00 moves.

For a sand plotter - allows paths to run forwards or reversed.
"""

import math
import re
from itertools import takewhile
from more_itertools import split_when

import gcodeutils.gcoder
from gcodeutils.gcoder import GCode

# Allow whitespace between axis name and coordinate
gcodeutils.gcoder.gcode_exp = re.compile(
    "\([^\(\)]*\)|^\(.*\)$|;.*|[/\*].*\n|([%s])\s*([-+]?[0-9]*\.?[0-9]*)"
    % gcodeutils.gcoder.to_parse
)

pattern = GCode(open("diffuse_0001_half.gcode"))


def is_g0(line):
    return line.command == "G00"


def not_g0(line):
    return line.command != "G00"


class Segment:
    def __init__(self, lines):
        self.lines = lines
        self.startpos = self._startpos()
        self.endpos = self._endpos()

    def _startpos(self):
        x = None
        y = None
        for line in self.lines:
            if x is None:
                x = line.x
            if y is None:
                y = line.y
            if x is not None and y is not None:
                return (x, y)

    def _endpos(self):
        x = None
        y = None
        for line in reversed(self.lines):
            if x is None:
                x = line.x
            if y is None:
                y = line.y
            if x is not None and y is not None:
                return (x, y)


class ReversedSegment:
    def __init__(self, segment):
        self.segment = segment

    @property
    def lines(self):
        return reversed(self.segment.lines)

    @property
    def startpos(self):
        return self.segment.endpos

    @property
    def endpos(self):
        return self.segment.startpos


# since we don't have code to properly reverse arcs, convert them to line segments.
#
for line in pattern.lines:
    if line.command in ("G02", "G03"):
        line.raw = "G01" + line.raw[3:]

parts = split_when(pattern.lines, lambda a, b: not_g0(a) and is_g0(b))

preamble = next(parts)

segments = []
for segment in parts:
    seg = Segment(segment)
    if seg.startpos and seg.endpos:
        segments.append(seg)
        segments.append(ReversedSegment(seg))


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
