#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import argparse
import traceback

if hasattr(yaml, 'warnings'):
    yaml.warnings({'YAMLLoadWarning': False})


def main():
    parser = argparse.ArgumentParser(description="Check if clang tool should be run according to settings file.")
    parser.add_argument("-project-name", help="CMake project name for whitelist checking.")
    parser.add_argument("-settings-file", help="YAML settings file for cmake_clang_tools that should be checked.")
    parser.add_argument("-trigger-file", help="CMake trigger file that shall be generated.")
    parser.add_argument("-tool-name", help="Name of the clang tool (clang_tidy/clang_format)")
    args = parser.parse_args()

    try:
        # Check if project is contained in whitelist and clang tool should be ran
        with open(args.settings_file, 'r') as settingsFile:
            def is_in_list(list_name: str, settings: dict, or_is_empty: bool):
                current_list = list()
                if list_name in settings:
                    current_list = settings[list_name]
                return (args.project_name in current_list) or (or_is_empty and not current_list)

            yaml_settings = yaml.load(settingsFile)
            run_clang_tool = yaml_settings['run_' + args.tool_name]

            if run_clang_tool \
                    and is_in_list('whitelist', yaml_settings, True) \
                    and not is_in_list('blacklist', yaml_settings, False):
                with open(args.trigger_file, 'w') as triggerFile:
                    triggerFile.write('INIT')
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
