#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import argparse
import traceback
from shutil import copyfile

if hasattr(yaml, 'warnings'):
    yaml.warnings({'YAMLLoadWarning': False})


def main():
    parser = argparse.ArgumentParser(description="Check if clang tool should be run according to settings file.")
    parser.add_argument("-project-name", help="CMake project name for whitelist checking.")
    parser.add_argument("-settings-file", help="YAML settings file for cmake_clang_tools that should be checked.")
    parser.add_argument("-settings-file-cached", help="Cached YAML settings file to detect changes.")
    parser.add_argument("-trigger-file", help="CMake trigger file that shall be generated.")
    parser.add_argument("-tool-name", help="Name of the clang tool (clang_tidy/clang_format)")
    args = parser.parse_args()

    try:
        with open(args.settings_file, 'r') as settingsFile:
            with open(args.settings_file_cached, 'r') as settingsFileCached:
                def has_list_changed(list_name: str, current_settings: dict, cached_settings: dict):
                    current_list = list()
                    if list_name in current_settings:
                        current_list = current_settings[list_name]
                    in_current_list = args.project_name in current_list
                    cached_list = list()
                    if list_name in cached_settings:
                        cached_list = cached_settings[list_name]
                    in_cached_list = args.project_name in cached_list
                    return in_current_list != in_cached_list

                yaml_settings = yaml.load(settingsFile)
                yaml_settings_cached = yaml.load(settingsFileCached)
                run_changed = yaml_settings['run_' + args.tool_name] != yaml_settings_cached['run_' + args.tool_name]

                if run_changed \
                        or has_list_changed('whitelist', yaml_settings, yaml_settings_cached) \
                        or has_list_changed('blacklist', yaml_settings, yaml_settings_cached):
                    open(args.trigger_file, 'w').close()
                copyfile(args.settings_file, args.settings_file_cached)
        exit(0)

    except Exception as e:
        print("Exception raised:")
        print("    " + str(e))
        print('-'*60)
        traceback.print_exc()
        print('-'*60)
        exit(-2)


if __name__ == "__main__":
    main()
