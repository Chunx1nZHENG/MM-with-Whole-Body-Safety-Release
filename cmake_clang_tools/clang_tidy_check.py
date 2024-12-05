#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###
# Authors:  Gabriel Hottiger
# Date:     20.02.2018
###

import argparse
import os
import subprocess
import yaml, json
import sys
import re
from itertools import chain
import traceback

if hasattr(yaml, 'warnings'):
    yaml.warnings({'YAMLLoadWarning': False})

__author__ = "Gabriel Hottiger"
__version__ = "0.1"

def check_init(trigger_file):
    with open(trigger_file, 'r') as f:
        return f.readline() == "INIT"

# Call clang tidy with given args
def execute_clang_tidy(executable, files, config_file, build_directory, header_filter, header_dirs, exclude_header_dirs, error, fix, verbose):

    # Check wheter search file should be attempted
    json_config = ""
    if config_file != "file":
        # Load yaml from file
        with open(config_file, 'r') as handle:
            yaml_config = yaml.load(handle)
        json_config = json.dumps(yaml_config)
        # Replace double quotes with single quotes
        idx = json_config.find("Checks") + 7
        json_config=json_config[:idx-1] + json_config[idx:].replace("\"", "\'", 2);
        # Remove remaining double quotes
        json_config=json_config.replace("\"", "")
        json_config = json.dumps(json_config)

    # Make list out of string
    header_dirs = header_dirs.split()
    if exclude_header_dirs:
    	exclude_header_dirs = exclude_header_dirs.split()
    #print(header_dirs)
    #print(exclude_header_dirs)

# Build header filter (this overwrites the header filter if specified)
    if header_dirs:
        if exclude_header_dirs:
            # Build a regular expression
            regEx = "^(?!.*" + os.path.join(exclude_header_dirs[0],'');
            for i in range(1, len(exclude_header_dirs)):
                regEx += ("|.*" + os.path.join(exclude_header_dirs[i],''));
            regEx +=").*"
            pattern = re.compile(regEx)

            # Walk through header_dirs and select matches
            header_filter = ""
            for root, directories, filenames in chain.from_iterable(os.walk(path) for path in header_dirs):
                for filename in filenames:
                    if re.match(pattern, os.path.join(root,filename)):
                        header_filter += (os.path.join(root,filename) + "|")
            header_filter = header_filter[:-1];
        else:
            header_filter = ""
            for dir in header_dirs:
                header_filter += (dir + "/.*|");
            header_filter = header_filter[:-1];

    # TODO do it the proper way without shell
    # clang_tidy_args = [executable]
    # clang_tidy_args.append("-quiet")
    # clang_tidy_args.append("--config={}".format(json.dumps(json_config)))
    # clang_tidy_args.append("-p={}".format(build_directory))
    # clang_tidy_args.append("-header-filter={}".format(header_filter))
    # clang_tidy_args.append("-export-fixes={}/clang-tidy-fixes.yaml".format(build_directory))
    # for file in files:
    #     clang_tidy_args.append(os.path.basename(file))

    # Add everything as a single string
    clang_tidy_args = executable + " --config={}".format(json_config) + \
        " -p={}".format(build_directory) + " -header-filter=\'{}\'".format(header_filter) + \
        " -export-fixes={}/clang-tidy-fixes.yaml".format(build_directory) + \
        " -extra-arg=-w";
    if error:
        clang_tidy_args += " -warnings-as-errors='*'";
    if fix:
        clang_tidy_args += " --fix";
    for file in files:
        clang_tidy_args += " " + os.path.abspath(file);
    # print( clang_tidy_args )
    # # Call clang-tidy
    stream = sys.stderr if verbose else sys.stdout
    return subprocess.call(clang_tidy_args, cwd=os.path.dirname(file), shell=True, stdout=stream, stderr=sys.stdout);

def main():
    parser = argparse.ArgumentParser(description="C/C++ style check using clang-tidy")

    # clang tidy executable
    parser.add_argument("-clang-tidy-executable",
                        default="clang-tidy",
                        help="Clang-tidy executable (default is '%(default)s').")

    # cmake project name
    parser.add_argument("-project",
                        default="",
                        help="CMake project name for white-listing (default is empty).")

    # .clang-tidy path
    parser.add_argument("-config-file", default=".clang_tidy",
                        help="YAML file containing the configuration (default is '%(default)s').")

    # Trigger file path
    parser.add_argument("-trigger-file", default=None,
                        help="Trigger file, must contain INIT to run (default is '%(default)s').")

    # Build directory
    parser.add_argument("-build-directory",
                        help="Directory where compile commands are located (fixes are exported to this directory).")

    # Header dirs
    parser.add_argument("-header-dirs",
                        help="Space separated list of header directories (default is empty).")

    # Exclude header dirs
    parser.add_argument("-exclude-header-dirs",
                        help="Space separated list of excluded header directories (default is empty). \
                             Only applied if header-dirs are set.")

    # Header filter
    parser.add_argument("-header-filter", default=".*",
                        help="Header filter to overload specifications from the config_file (default is '%(default)s'). \
                                Only applied if no header-dirs are set.")

    # Warnings as Errors
    parser.add_argument("--error",
                        action="store_true",
                        help="All warnings are treated as errors.")

    # Inplace-fixing
    parser.add_argument("--fix",
                        action="store_true",
                        help="Apply the fixes to the issues discovered by clang-tidy (not recommended).")

    # Verbose
    parser.add_argument("--verbose",
                        action="store_true",
                        help="Output to stderr.")

    # Files or directory to check
    parser.add_argument("files", nargs="+", help="Paths to the files that should be checked.")

    args = parser.parse_args()

    try:
        if args.trigger_file and check_init(args.trigger_file):
            result = execute_clang_tidy(args.clang_tidy_executable, args.files, args.config_file, args.build_directory, args.header_filter,
                                        args.header_dirs, args.exclude_header_dirs, args.error, args.fix, args.verbose)
            exit(result)
        else:
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
