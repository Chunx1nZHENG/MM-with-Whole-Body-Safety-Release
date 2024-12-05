#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###
# Authors:  cloderic, Gabriel Hottiger
# Date:     20.02.2018
###

import argparse
import glob
import os
import subprocess
import xml.etree.ElementTree as ET
from collections import namedtuple
import yaml, json, sys
import traceback

if hasattr(yaml, 'warnings'):
    yaml.warnings({'YAMLLoadWarning': False})

Replacement = namedtuple("Replacement", "offset length text")
Error = namedtuple("Error", "line column found expected")

__author__ = "github.com/cloderic"
__version__ = "0.3"

def check_init(trigger_file):
    with open(trigger_file, 'r') as f:
        return f.readline() == "INIT"

def replacements_from_file(executable, file, json_config, fix):
    replacements = []
    clang_format_args = [executable]
    clang_format_args.append("-style={}".format(json_config))
    if fix:
        clang_format_args.append("-i")
    else:
        clang_format_args.append("-output-replacements-xml")

    clang_format_args.append(os.path.basename(file))
    replacement_xml = subprocess.check_output(clang_format_args, cwd=os.path.dirname(file))

    if replacement_xml:
        replacement_xml_root = ET.XML(replacement_xml)
        for replacement_item in replacement_xml_root.findall('replacement'):
            replacements.append(Replacement(
                offset=int(replacement_item.attrib["offset"]),
                length=int(replacement_item.attrib["length"]),
                text=replacement_item.text
            ))

    return replacements

def errors_from_replacements(file, replacements=[]):
    errors = []

    lines = [0]  # line index to character offset
    file_content = ""
    for line in open(file, "r"):
        file_content += line
        lines.append(lines[-1] + len(line))

    for line_index, line_offset in enumerate(lines[:-1]):
        while (len(replacements) > 0 and
               lines[line_index + 1] > replacements[0].offset):
            replacement = replacements.pop(0)
            errors.append(Error(
                line=line_index,
                column=replacement.offset - line_offset,
                found=file_content[replacement.offset:replacement.offset +
                                   replacement.length],
                expected=replacement.text if replacement.text else ""
            ))

        if len(replacements) == 0:
            break

    return errors


def clang_format_check(executable, files, json_config, fix):
    error_count = 0
    file_errors = dict()

    for file in files:
        replacements = replacements_from_file(executable, file, json_config, fix)
        errors = errors_from_replacements(file, replacements)
        error_count += len(errors)
        file_errors[file] = errors
    return error_count, file_errors


def print_error_report(error_count, file_errors, error_assert, fix, verbose):
    stream = sys.stderr if verbose else sys.stdout

    if error_count == 0:
        if fix:
            print ("All errors were fixed.")
        else:
            print ("No format error found.")
    else:
        for file, errors in file_errors.items():
            for error in errors:
                print ("\033[1m\033[97m{}:{}:{}: ".format(os.path.abspath(file),error.line + 1, error.column + 1), end='', file=stream)
                if error_assert:
                    print ('\033[1m\033[91merror:', end='', file=stream)
                else:
                    print ('\033[1m\033[35mwarning:', end='', file=stream)
                print ('\033[97m clang-format\033[0m', file=stream)
                print ('  Found: \'{}\', Expected: \'{}\', CharacterDiff: \'{}\''.format("\\n".join(error.found.split("\n")),\
                    "\\n".join(error.expected.split("\n")), abs(len(error.found)- len(error.expected))), file=stream)

def main():
    parser = argparse.ArgumentParser(
        description="C/C++ formatting check using clang-format")

    # clang-format executable
    parser.add_argument("-clang-format-executable",
                        default="clang-format",
                        help="Clang-format executable (default is '%(default)s').")

    # .clang-format path
    parser.add_argument("-config-file",
                        default=".clang-format",
                        help="Clang format configuration YAML file (default is '%(default)s').")

    # Trigger file path
    parser.add_argument("-trigger-file", default=None,
                        help="Trigger file, must contain INIT to run (default is '%(default)s').")

    # Warnings as Errors
    parser.add_argument("--error",
                        action="store_true",
                        help="All warnings are treated as errors.")

    # Inplace-fixing
    parser.add_argument("--fix",
                        action="store_true",
                        help="Fix the foormatting issues of the files in-place.")

    # Verbose
    parser.add_argument("--verbose",
                       action="store_true",
                       help="Output to stderr.")

    # Files or directory to check
    parser.add_argument("files", nargs="+", help="Paths to the files that should be checked.")

    args = parser.parse_args()

    try:
        if args.trigger_file and check_init(args.trigger_file):
             # Load yaml from file
             with open(args.config_file, 'r') as handle:
                 yaml_config = yaml.load(handle)

             # globing the file paths
             files = set()
             for pattern in args.files:
                 for file in glob.iglob(pattern):
                     files.add(os.path.relpath(file))
             file_list = list(files)

             # Run clang-format
             error_count, file_errors = clang_format_check(args.clang_format_executable, file_list, json.dumps(yaml_config), args.fix)
             print_error_report(error_count, file_errors, args.error, args.fix, args.verbose)

             # Return false if errors occured
             if args.error:
                 exit(error_count)

        exit(0)

    except Exception as e:
        print ("Exception raised:")
        print ("    " + str(e))
        print ('-'*60)
        traceback.print_exc()
        print ('-'*60)
        exit(-2)

if __name__ == "__main__":
    main()
