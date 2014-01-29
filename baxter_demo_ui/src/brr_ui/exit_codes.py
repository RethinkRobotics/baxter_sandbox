import sys

EXIT_SHUTDOWN = "EXIT_SHUTDOWN"
EXIT_REBOOT = "EXIT_REBOOT"

def exit_with_return_code(return_code):
    with open("/var/tmp/hlr/demo_exit_code", "w") as exit_code_file:
        exit_code_file.write(return_code)
        sys.exit(0)
