import os
from subprocess import Popen


class Terminal:
    def __init__(self, identifier):
        """
        Initialize a new xterm terminal.
        :param identifier: terminal identifier.
        """

        script_path = os.path.realpath(__file__)[:-20]

        self._id = id
        self._pipe_path = script_path+"pipes/pipe_xterm_"+str(identifier)

        if not os.path.exists(self._pipe_path):
            try:
                os.mkfifo(self._pipe_path)
            except OSError as e:
                print("Failed to create FIFO: {}".format(e))

        Popen(['xterm', '-e', 'tail -f %s' % self._pipe_path])

        with open(self._pipe_path, "w") as p:
            p.write("Terminal for agent id {} ready.\n".format(identifier))

    def write(self, message):
        with open(self._pipe_path, "w") as p:
            p.write(message+"\n")


