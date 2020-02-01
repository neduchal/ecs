#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json

def load(filename):
    """
        Nacteni komentovaneho JSON souboru

        :param filename: cesta k souboru
        :type filename: string
    """

    fp = open(filename, 'r')

    lines = fp.readlines()
    data = "";
    for line in lines:
        linestrip = line.lstrip()
        if linestrip[0:1] == '#':
            continue
        else:
            data = data + line

    data = json.loads(data)

    fp.close()
    return data
