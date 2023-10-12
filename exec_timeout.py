#! /usr/bin/env python3
import sys
import subprocess

def exec_timeout(command, timeout):
    command = command.split()
    p = subprocess.Popen(command)
    try:
        p.wait(timeout)
    except subprocess.TimeoutExpired:
        p.kill()

def fix_command(command):
    C = ''
    for i in range(len(command)):
        c = command[i]
        C += c
        if i != len(command) - 1:
            C += ' '
    return C


if __name__ == '__main__':
    args = sys.argv
    timeout = int(args[1])
    command = args[2:]
    command = fix_command(command)
    print("command:")
    print(command)
    exec_timeout(command, timeout)
