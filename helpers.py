from platform  import system as system_name  # Returns the system/OS name
from subprocess import call as system_call  # Execute a shell command


class HelpersLINQ:

    @staticmethod
    def distinct(sequence):
        seen = set()
        for s in sequence:
            if not s in seen:
                seen.add(s)
                yield s


class HelpersROS:
    # Process response of type array = [ [a, [a1, a2] ] , [b, [b1, b2] ] ]
    @staticmethod
    def process_line(array_object):
        if len(array_object) == 0:
            return None

        topic_name = array_object[0]
        node_names = (HelpersLINQ.distinct(array_object[1]))

        return [topic_name, node_names]


class HelpersNetWorking:
    @staticmethod
    def ping(host):
        ret = None
        # Ping command count option as function of OS
        param = '-n' if system_name().lower()=='windows' else '-c'
        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', host]
        with open("/dev/null", "w+") as f:
            ret = system_call(command, stdout=f) == 0

        return ret