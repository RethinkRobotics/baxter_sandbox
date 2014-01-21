# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from subprocess import Popen, PIPE, STDOUT
import signal

class TimeoutException(Exception):
    pass

## Creates a python decorator to add a timeout to a function.
#  Displays the "default" message if the timeout
#  is reached before a return on the funciton.
#  Returns a function that will attempt to run the decorated function but will
#  return the "default" value if nothing is returned in "timeout_time" seconds.
def timeout(timeout_time, default):
    def timeout_function(f):
        def f2(*args):
            def timeout_handler(signum, frame):
                raise TimeoutException()

            old_handler = signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(timeout_time)
            try:
                retval = f(*args)
            except TimeoutException:
                return default
            finally:
                signal.signal(signal.SIGALRM, old_handler)
            signal.alarm(0)
            return retval
        return f2
    return timeout_function

## Simple function to execute a linux shell command
#  @param command The bash command to be run
#  @param quiet Tells the function no to print the output of the command
#  @param get_output Tells the script to return the output of the command
def mk_process(command, quiet=False, get_output=False, shell=True):
    if shell==True:
        process = Popen(command, shell = True, stdout = PIPE,
                               stderr = STDOUT)
    else:
        process = Popen(command, shell = False, stdout = PIPE,
                               stderr = STDOUT)
    stdout, stderr = process.communicate()
    if quiet == False:
        print stdout
    if get_output:
        return stdout
    else:
        return process.returncode

## Equivalent of "mkdir -p" in linux shell
def mkdirs(path):
    try:
        os.makedirs(path)
    except:
        print "Folder \"%s\" already exists" % path

# Pretty much the same as mk_process, but in a class so that
#   the user can call the access the process after instantiation
class ros_process():
    def __init__(self, command, quiet=True, get_output=False, shell=True):
        if shell==True:
            self.process = Popen(command, shell=True, stdout=None, 
                                        stdin=PIPE, stderr=STDOUT)
        else:
            self.process = Popen(command, shell=False, stdout=None, 
                                        stdin=PIPE, stderr=STDOUT)
        self.quiet = quiet
        self.get_output = get_output

    def write(self, stdin):
        self.process.stdin.write(stdin)

    def run(self):
        stdout, stderr = self.process.communicate()
        if self.quiet == False:
            print stdout
        if self.get_output:
            return stdout
        else:
            return self.process.returncode

## Example program showing how to use the timeout decorator
@timeout(3, "You didn't type anything")
def do_stuff():
    raw_input("Type something, or don't...:__________\n")
    return ""

if __name__ == "__main__":
    print do_stuff()
